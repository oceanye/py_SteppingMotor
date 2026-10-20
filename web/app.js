import {api, num, act, notice} from './common.js';
import {updateCoordinated} from './logical-motors.js';
import {updateSteppers} from './legacy-steppers.js';

// These subsystems are NOT the four X42S roles.
const motors=document.getElementById('motors');
for(let a=0;a<2;a++) motors.insertAdjacentHTML('beforeend',`<div class="card"><div class="title"><span>GEAR / FOC 轴 ${a+1}</span><span id="mstate${a}" class="state">--</span></div><div class="fields"><label>控制模式<select id="mode${a}"><option>GEAR</option><option>FOC</option></select></label><label>目标角 (°)<input id="target${a}" type="number" value="0" min="-3600" max="3600" step="1"></label></div><div class="buttons"><button class="enable" data-motor="enable" data-axis="${a}">使能</button><button data-motor="zero" data-axis="${a}">归零</button><button class="stop" data-motor="disable" data-axis="${a}">失能</button><button class="go wide" data-motor="target" data-axis="${a}">发送目标角</button></div></div>`);
motors.addEventListener('click',e=>{
  const b=e.target.closest('button[data-motor]');
  if(!b || b.disabled)return;
  const axis=Number(b.dataset.axis),action=b.dataset.motor;
  act(()=>{
    const body={mode:document.getElementById(`mode${axis}`).value,axis,action};
    if(action==='target')body.target_deg=num(`target${axis}`,-3600,3600);
    return api('/api/motor',body);
  },`GEAR / FOC 轴 ${axis+1}：${action}`);
});

let holding=null,leaseTimer=null,trackPending=null;
let connected=false,refreshInFlight=false;
let modeInitialized=false;
const editedModes=new Set();
for(let i=0;i<2;i++)document.getElementById(`mode${i}`).addEventListener('change',()=>editedModes.add(i));
function cancelLease(){holding=null;clearTimeout(leaseTimer);leaseTimer=null;}
function stopTrack(show=false){
  cancelLease();
  const pending=trackPending;
  act(()=>api('/api/track',{action:'stop'}),show?'轨道停车命令已确认':null);
  // Reassert STOP if a run request finishes after release/blur.
  if(pending)pending.finally(()=>act(()=>api('/api/track',{action:'stop'}),null));
}
function startTrack(action,button,event){
  event.preventDefault();
  if(holding || !connected || button.disabled)return;
  let pwm;
  try{pwm=num('trackPwm',1,100);}catch(e){notice(e.message,true);return;}
  const token={action};holding=token;
  try{button.setPointerCapture(event.pointerId);}catch(_e){}
  const send=async()=>{
    if(holding!==token)return;
    const pending=api('/api/track',{action,pwm,lease_ms:1000});
    // Always-settled cleanup promise prevents unhandled rejections.
    const settled=pending.catch(()=>{});trackPending=settled;
    try{
      await pending;
      if(holding===token){
        document.getElementById('trackState').textContent=action==='forward'?'前进':'后退';
        leaseTimer=setTimeout(send,350);
      }
    }catch(e){
      if(holding===token){notice(e.message,true);stopTrack();}
    }finally{if(trackPending===settled)trackPending=null;}
  };
  send();
}
for(const [id,dir] of [['trackFwd','forward'],['trackRev','reverse']]){
  const button=document.getElementById(id);
  button.addEventListener('pointerdown',e=>startTrack(dir,button,e));
  for(const event of ['pointerup','pointercancel','lostpointercapture'])
    button.addEventListener(event,()=>{if(holding)stopTrack();});
}
document.getElementById('trackStop').addEventListener('click',()=>stopTrack(true));
window.addEventListener('blur',()=>{if(holding)stopTrack();});
window.addEventListener('pagehide',()=>{if(holding)stopTrack();});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&holding)stopTrack();});
document.getElementById('estop').addEventListener('click',()=>{
  cancelLease();
  act(async()=>{
    const result=await api('/api/estop',{});
    if(result.confirmed!==true)throw new Error('急停未获固件确认，请检查现场并使用硬件急停');
    return result;
  },'固件已确认全部软件停车命令');
});

function setConnected(value){
  connected=value;
  document.getElementById('dot').classList.toggle('ok',value);
  for(const button of document.querySelectorAll('#motors button,#steppers button,#trackFwd,#trackRev'))
    button.disabled=!value;
  if(!value&&holding)stopTrack();
}
function stateText(value){return value==null?'--':typeof value==='object'?JSON.stringify(value):String(value);}
async function refresh(){
  if(refreshInFlight)return;
  refreshInFlight=true;
  try{
    const s=await api('/api/status');
    if(typeof s.connected!=='boolean' || !Array.isArray(s.logical_motors))throw new Error('状态响应不完整');
    document.getElementById('legacySteppers').hidden=s.legacy_stepdir_enabled!==true;
    updateSteppers(s);
    setConnected(s.connected);
    document.getElementById('conn').textContent=s.connected?'串口已连接':'服务在线 · 串口未连接';
    for(let i=0;i<2;i++)document.getElementById(`mstate${i}`).textContent=stateText(s.motors?.[i]);
    // Polling must not overwrite a user's selected command mode.
    document.getElementById('firmwareMode').textContent=`固件模式：${s.mode||'未知'}`;
    if(!modeInitialized && ['GEAR','FOC'].includes(s.mode)){
      for(let i=0;i<2;i++)if(!editedModes.has(i))document.getElementById(`mode${i}`).value=s.mode;
      modeInitialized=true;
    }
    if(s.track!==undefined&&!holding)document.getElementById('trackState').textContent=stateText(s.track);
    updateCoordinated(s.coordinated_control||{},s.logical_motors,s.connected);
  }catch(_e){
    setConnected(false);
    document.getElementById('conn').textContent='网页服务连接中断';
    document.getElementById('firmwareMode').textContent='固件模式未知（连接中断）';
    updateCoordinated({},[],false);
    for(let i=0;i<2;i++)document.getElementById(`mstate${i}`).textContent='状态未知';
    document.getElementById('trackState').textContent='状态未知（请确认现场停车）';
  }finally{refreshInFlight=false;}
}
setConnected(false);
refresh();setInterval(refresh,1000);
