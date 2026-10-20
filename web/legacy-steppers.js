import {api, num, act, notice, formatNumber} from './common.js';

const SPEED_PRESETS=[0.05,0.1,0.2,0.3,0.5,0.6,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,8,10,15,20,30];
const PULSE_RATE_WARN_PPS=1000;
const POSITION_SOURCE='host_pulse_accounting';
const speedPresetList=document.getElementById('speedPresets');
for(const value of SPEED_PRESETS){const option=document.createElement('option');option.value=String(value);speedPresetList.appendChild(option);}
const steppers=document.getElementById('steppers'),stepperGroup=document.getElementById('stepperGroup');
const dirtyAxes=new Set();
let stepperTopology=[];
function fallbackTopology(){return Array.from({length:30},(_v,axis)=>axis<6?{axis,label:['左侧直','右侧直','左侧转','右侧转','左开合','右开合'][axis],controller:'esp32',node:null,local_axis:axis,pins:null}:{axis,label:`P${Math.floor((axis-6)/4)+1}-${(axis-6)%4+1}`,controller:'pico',node:Math.floor((axis-6)/4)+1,local_axis:(axis-6)%4,pins:null});}
function groupKey(t){return t.controller==='esp32'?'esp32':`pico-${t.node}`;}
function groupLabel(t){return t.controller==='esp32'?'ESP32 本地（6 轴）':`Pico ${t.node}（4 轴）`;}
function routeLabel(t){if(t.controller==='esp32'){const p=t.pins;return p?`本地 · PUL GPIO${p.pulse} / DIR GPIO${p.direction}`:'ESP32 本地';}return `RS485 · Pico ${t.node} · 本地轴 ${Number(t.local_axis)+1}`;}
function esc(v){return String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function axisCard(axis){return steppers.querySelector(`[data-axis-card="${axis}"]`);}
function axisConfigIsDirty(axis){return dirtyAxes.has(axis);}
function setAxisConfigDirty(axis,dirty=true){
  if(dirty)dirtyAxes.add(axis);else dirtyAxes.delete(axis);
  axisCard(axis)?.classList.toggle('config-dirty',dirty);
}
function syncAxisModeUi(axis,mode){
  const rotary=mode==='rotary',unit=rotary?'°':'mm';
  const unitNode=document.getElementById(`sunit${axis}`),speedUnit=document.getElementById(`sunits${axis}`),noun=document.getElementById(`snoun${axis}`),lead=document.getElementById(`slead${axis}`);
  if(unitNode)unitNode.textContent=unit;
  if(speedUnit)speedUnit.textContent=`${unit}/s`;
  if(noun)noun.textContent=rotary?'角度':'距离';
  if(lead){lead.disabled=rotary;lead.title=rotary?'旋转模式不使用导程':'';}
}
function setAxisConfigForm(axis,source){
  const values=[['smode','mode'],['sppr','pulse_per_rev'],['sgr','gear_ratio'],['slead','lead_mm']];
  for(const [prefix,key] of values){const node=document.getElementById(`${prefix}${axis}`);if(node&&source[key]!==undefined&&source[key]!==null)node.value=source[key];}
  const mode=document.getElementById(`smode${axis}`)?.value||'linear';
  syncAxisModeUi(axis,mode);
}
function axisConfigPayload(axis){
  return {
    axis,
    mode:document.getElementById(`smode${axis}`).value,
    pulse_per_rev:num(`sppr${axis}`,1,10000),
    gear_ratio:num(`sgr${axis}`,.001,1000),
    lead_mm:num(`slead${axis}`,.01,100)
  };
}
function requestedPulseRate(axis,speed){
  const mode=document.getElementById(`smode${axis}`).value;
  const ppr=num(`sppr${axis}`,1,10000),ratio=num(`sgr${axis}`,.001,1000);
  const pulsesPerUnit=mode==='rotary'?ppr*ratio/360:ppr*ratio/num(`slead${axis}`,.01,100);
  return speed*pulsesPerUnit;
}
function statusValue(axisStatus,key,legacyKey){return axisStatus[key]??axisStatus[legacyKey];}
function updateAxisSummary(axis,axisStatus){
  const node=document.getElementById(`sdetail${axis}`);
  if(!node)return;
  const unit=axisStatus.unit||(axisStatus.mode==='rotary'?'°':'mm');
  const position=statusValue(axisStatus,'position','position_mm');
  const minimum=statusValue(axisStatus,'travel_min','travel_min_mm');
  const maximum=statusValue(axisStatus,'travel_max','travel_max_mm');
  const trust=axisStatus.position_trusted?'可信':'不可信，需校准';
  const motion=axisStatus.continuous?'连续运行':axisStatus.in_progress?'运动中':'停止';
  const mode=axisStatus.mode==='rotary'?'旋转':'直线';
  node.textContent=`位置 ${formatNumber(position)} ${unit}（${trust}） · ${motion}\n限位 min ${formatNumber(minimum)} / max ${formatNumber(maximum)} ${unit}\n已应用参数 ${mode} · PPR ${formatNumber(axisStatus.pulse_per_rev)} · GR ${formatNumber(axisStatus.gear_ratio)} · 导程 ${formatNumber(axisStatus.lead_mm)} mm/转`;
}
function moveResultNotice(result,label){
  const requested=formatNumber(result.requested_pulse_rate_pps,0);
  const effective=formatNumber(result.effective_pulse_rate_pps,0);
  return result.speed_clamped
    ?`轴 ${label} 命令已发送；请求 ${requested} pps，已钳位为约 ${effective} pps`
    :`轴 ${label} 命令已发送（约 ${requested} pps）`;
}
function buildSteppers(topology){
  const selectedGroup=stepperGroup.value;
  dirtyAxes.clear();
  stepperTopology=topology.length?topology:fallbackTopology();steppers.textContent='';stepperGroup.textContent='';
  const groups=[];for(const t of stepperTopology){const key=groupKey(t);if(!groups.some(g=>g.key===key))groups.push({key,label:groupLabel(t)});}
  for(const g of groups)stepperGroup.insertAdjacentHTML('beforeend',`<option value="${esc(g.key)}">${esc(g.label)}</option>`);
  if(groups.some(group=>group.key===selectedGroup))stepperGroup.value=selectedGroup;
  for(const t of stepperTopology){
    const a=Number(t.axis);
    steppers.insertAdjacentHTML('beforeend',`<div class="card" data-axis-card="${a}" data-group="${esc(groupKey(t))}"><div class="title"><span>轴 ${esc(t.label||a)}<span class="dirty-badge">配置未应用</span></span><span id="sstate${a}" class="state">--</span></div><span class="route">${esc(routeLabel(t))}</span><div id="sdetail${a}" class="axis-summary">正在读取轴状态…</div><div class="fields"><label>模式<select id="smode${a}" data-axis-config="${a}"><option value="linear">直线 (mm)</option><option value="rotary">旋转 (°)</option></select></label><label>脉冲/转<input id="sppr${a}" data-axis-config="${a}" type="number" value="200" min="1" max="10000" step="1"></label><label>减速比<input id="sgr${a}" data-axis-config="${a}" type="number" value="1" min="0.001" max="1000" step="0.01"></label><label>导程 (mm/转)<input id="slead${a}" data-axis-config="${a}" type="number" value="1" min="0.01" max="100" step="0.1"></label><label><span id="snoun${a}">距离</span> (<span id="sunit${a}">mm</span>)<input id="dist${a}" type="number" value="10" min="0.001" max="100000" step="0.1"></label><label>速度 (<span id="sunits${a}">mm/s</span>)<input id="speed${a}" type="number" value="3" min="0.001" max="10000" step="any" list="speedPresets"></label></div><div class="buttons"><button class="go" data-step="reverse" data-axis="${a}">反向</button><button class="stop" data-step="stop" data-axis="${a}">停止</button><button class="go" data-step="forward" data-axis="${a}">正向</button></div><button class="enable wide" data-step="applycfg" data-axis="${a}">应用轴配置</button></div>`);
    syncAxisModeUi(a,'linear');
  }
  showStepperGroup();document.getElementById('topologyNote').textContent=`${stepperTopology.length} 轴 · ESP32 本地 + 6 个普通 Pico`;
}
function showStepperGroup(){const selected=stepperGroup.value;for(const card of steppers.querySelectorAll('[data-axis-card]'))card.hidden=card.dataset.group!==selected;}
function markAxisConfigEdited(event){
  const field=event.target.closest('[data-axis-config]');
  if(!field)return;
  const axis=Number(field.dataset.axisConfig);
  setAxisConfigDirty(axis);
  if(field.id===`smode${axis}`)syncAxisModeUi(axis,field.value);
}
stepperGroup.addEventListener('change',showStepperGroup);
steppers.addEventListener('input',markAxisConfigEdited);
steppers.addEventListener('change',markAxisConfigEdited);

steppers.addEventListener('click',e=>{
  const b=e.target.closest('button[data-step]');
  if(!b)return;
  const axis=Number(b.dataset.axis),direction=b.dataset.step,label=stepperTopology.find(t=>Number(t.axis)===axis)?.label??axis;
  if(direction==='stop'){
    act(()=>api('/api/stepper/stop',{axis}),`轴 ${label} 已停止`);
    return;
  }
  if(direction==='applycfg'){
    let submitted;
    act(
      ()=>{submitted=axisConfigPayload(axis);return api('/api/stepper/config',submitted);},
      `轴 ${label} 配置已应用`,
      result=>{setAxisConfigForm(axis,{...submitted,...result});setAxisConfigDirty(axis,false);}
    );
    return;
  }
  act(async()=>{
    if(axisConfigIsDirty(axis))throw new Error('该轴有尚未应用的配置，请先点击“应用轴配置”');
    const distance=num(`dist${axis}`,.001,100000),speed=num(`speed${axis}`,.001,10000);
    const requestedPps=requestedPulseRate(axis,speed),highRate=requestedPps>PULSE_RATE_WARN_PPS;
    if(highRate&&!window.confirm(`轴 ${label} 请求约 ${formatNumber(requestedPps,0)} 脉冲/秒，超过安全参考值 ${PULSE_RATE_WARN_PPS} pps。\n\n无加速曲线时可能失步，请确认模式、PPR、减速比和导程均正确。仍要执行吗？`))return null;
    const result=await api('/api/stepper/move',{axis,direction,distance_mm:distance,speed_mm_s:speed,confirm_high_rate:highRate});
    if(result.requested_pulse_rate_pps===undefined)result.requested_pulse_rate_pps=requestedPps;
    return result;
  },null,result=>{if(result)notice(moveResultNotice(result,label),false,result.speed_clamped?5200:3000);});
});


export function updateSteppers(s) {
    if(s.legacy_stepdir_enabled!==true){
      steppers.textContent='';stepperGroup.textContent='';stepperTopology=[];dirtyAxes.clear();return;
    }
    const incoming=s.topology?.axes||[];
    if(incoming.length&&JSON.stringify(incoming)!==JSON.stringify(stepperTopology))buildSteppers(incoming);
    const ss=s.steppers||[],axes=s.stepper_axes||[];
    for(const t of stepperTopology){
      const i=Number(t.axis),state=document.getElementById(`sstate${i}`),ax=axes[i];
      if(state)state.textContent=ss[i]==null?'--':String(ss[i]);
      if(!ax)continue;
      updateAxisSummary(i,ax);
      if(!axisConfigIsDirty(i))setAxisConfigForm(i,ax);
      else syncAxisModeUi(i,document.getElementById(`smode${i}`).value);
    }
}
