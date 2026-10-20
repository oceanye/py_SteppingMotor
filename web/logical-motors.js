import {formatNumber} from './common.js';

const LOGICAL_ROLES=[['Mup1','左侧升降','linear'],['Mr1','左侧旋转','rotary'],['Mup2','右侧升降','linear'],['Mr2','右侧旋转','rotary']];
const LOGICAL_STATE_TEXT={UNBOUND:'未绑定',INVALID:'配置无效',DISCONNECTED:'串口未连接',NODE_OFFLINE:'Pico 离线',NODE_UNKNOWN:'Pico 状态未知',HARDWARE_ESTOP:'硬件急停',SOFTWARE_ESTOP:'软件急停',ESTOP_UNCONFIRMED:'急停未确认',STOPPING:'停止中',STARTING:'启动中',CONTINUOUS:'连续运动',MOVING:'运动中',ABORTED_UNTRUSTED:'已中止/需校准',IDLE_UNTRUSTED:'空闲/需校准',IDLE:'空闲',X42S_BACKEND_PENDING:'已绑定·后端待接入',X42S_ONLINE:'X42S 在线',X42S_OFFLINE:'X42S 掉线',X42S_FAULT:'X42S 故障',X42S_STALE:'X42S 遥测超期'};
const BLOCKER_TEXT={logical_motor_bindings_incomplete_or_invalid:'四个逻辑角色尚未全部正确绑定',direction_and_zero_unverified:'方向与机械零位尚未实机确认',contact_feedback_unavailable:'接触状态反馈不可用',support_load_feedback_unavailable:'支撑载荷/电流反馈不可用',collision_model_unverified:'碰撞包络尚未验证',hardware_limit_chain_incomplete:'硬限位链不完整',coordinated_start_protocol_unavailable:'四轴协调启动协议尚未实现',pico_node_health_incomplete:'Pico 节点在线状态尚未完整缓存'};
const logicalMotors=document.getElementById('logicalMotors');
function buildLogicalMotors(){
  logicalMotors.textContent='';
  for(const [role,name,mode] of LOGICAL_ROLES){
    const visual=mode==='linear'?'<div class="linear-track"><span class="linear-fill"></span><span class="linear-marker"></span></div>':'<div class="dial"><span class="needle"></span></div>';
    logicalMotors.insertAdjacentHTML('beforeend',`<div class="card logical-card" data-logical-role="${role}"><div class="title"><span>${role} · ${name}</span><span class="logical-badge warn">未绑定</span></div><div class="logical-visual">${visual}</div><div><div class="logical-position">--</div><div class="logical-detail">未绑定物理执行器</div></div></div>`);
  }
}
function logicalBadgeClass(state){if(state==='IDLE'||state==='X42S_ONLINE')return 'good';if(['MOVING','CONTINUOUS','STARTING'].includes(state))return 'moving';if(['UNBOUND','NODE_UNKNOWN','IDLE_UNTRUSTED','ABORTED_UNTRUSTED','STOPPING','X42S_BACKEND_PENDING','X42S_STALE'].includes(state))return 'warn';return 'bad';}
function updateLogicalMotor(item){
  if(!LOGICAL_ROLES.some(([role])=>role===item.role))return;
  const card=logicalMotors.querySelector(`[data-logical-role="${item.role}"]`);if(!card)return;
  const badge=card.querySelector('.logical-badge'),positionNode=card.querySelector('.logical-position'),detail=card.querySelector('.logical-detail');
  const state=item.state||'INVALID';
  badge.textContent=LOGICAL_STATE_TEXT[state]||state;
  badge.className=`logical-badge ${logicalBadgeClass(state)}`;
  if(state==='X42S_ONLINE'){
    if(!item.position_trusted){badge.textContent='X42S 反馈待确认';badge.className='logical-badge warn';}
    else if(item.in_progress){badge.textContent='X42S 运动中';badge.className='logical-badge moving';}
    else{badge.textContent=item.x42s?.enabled?'X42S 已到位':'X42S 未使能';if(!item.x42s?.enabled)badge.className='logical-badge warn';}
  }
  const position=item.position,hasPosition=typeof position==='number'&&Number.isFinite(position),unit=item.unit||(item.required_mode==='rotary'?'°':'mm');
  positionNode.textContent=hasPosition?`${item.position_trusted?'':'历史 / 未确认 '}${formatNumber(position)} ${unit}`:'--';
  const binding=item.binding,isX42s=binding&&binding.kind==='x42s';
  const physical=isX42s?`${item.physical_label||`X42S ID${binding.driver_id}`} · 驱动器遥测`:(binding?`步进轴 ${binding.axis} · ${item.physical_label||'--'} · ${item.controller==='pico'?`Pico ${item.node}`:'ESP32 本地'}`:'未绑定物理执行器');
  let progress,modeLine;
  if(isX42s){
    const x=item.x42s||{},parts=[];
    if(typeof x.position_error==='number')parts.push(`电机轴误差 ${formatNumber(x.position_error,1)}°`);
    if(typeof x.speed==='number')parts.push(`电机轴速度 ${formatNumber(x.speed,1)} rpm`);
    if(x.fault)parts.push(`故障 ${x.fault}`);
    progress=parts.length?parts.join(' · '):(x.online?'无遥测字段':'无遥测');
    if(Number.isFinite(item.target_position))progress+=` · 指令目标 ${formatNumber(item.target_position)} ${unit}`;
    if(Number.isFinite(item.last_update_age_ms))progress+=` · 样本 ${item.last_update_age_ms} ms`;
    modeLine=item.position_source==='x42s_driver'?`驱动器反馈换算 · 单位 ${unit}`:'X42S 后端待接入';
  }else{
    progress=item.progress_percent==null?(item.in_progress?'运动中，暂无中间进度帧':'空闲'):`进度 ${formatNumber(item.progress_percent,1)}% · ${item.executed_steps}/${item.requested_steps} 脉冲`;
    if(typeof item.target_position==='number'&&Number.isFinite(item.target_position))progress+=` · 目标 ${formatNumber(item.target_position)} ${unit}`;
    modeLine=`${item.mode||item.required_mode} · ${item.position_trusted?'软件位置账本可信':'软件位置需校准'} · 非实测`;
  }
  detail.textContent=`${physical}\n${modeLine}\n${item.binding_error||progress}${item.stale?' · 数据陈旧':''}`;
  const drawable=hasPosition&&item.position_trusted===true;
  if(item.required_mode==='linear'){
    const fill=card.querySelector('.linear-fill'),marker=card.querySelector('.linear-marker'),minimum=item.travel_min,maximum=item.travel_max;
    const ranged=drawable&&typeof minimum==='number'&&Number.isFinite(minimum)&&typeof maximum==='number'&&Number.isFinite(maximum)&&maximum>minimum;
    const pct=ranged?Math.max(0,Math.min(100,100*(position-minimum)/(maximum-minimum))):0;
    fill.style.height=ranged?`${pct}%`:'0';marker.style.bottom=ranged?`${pct}%`:'0';marker.style.display=ranged?'block':'none';
    if(isX42s&&!ranged)detail.textContent+='\n未设有效行程范围，仅显示 mm 数值（非百分比）';
  }else{
    const needle=card.querySelector('.needle');needle.style.transform=drawable?`rotate(${((position%360)+360)%360}deg)`:'rotate(0deg)';needle.style.opacity=drawable?'1':'.25';
  }
}
function updateCoordinated(control,motors,connected=true){
  const ready=document.getElementById('coordinatedReadiness'),valid=!!control?.configuration_valid;
  const blockers=(control?.blockers||[]).map(code=>BLOCKER_TEXT[code]||code);
  ready.textContent=`绑定配置：${valid?'有效':'未完成或无效'} · 无人值守：未启用（步态以桌面预检为准）\n${blockers.join('；')}`;
  for(const [role,,mode] of LOGICAL_ROLES){
    const found=motors.find(item=>item&&item.role===role);
    const item=found?{...found}:{role,required_mode:mode,state:'INVALID',binding_error:'本次状态缺失，位置未知'};
    if(!connected){item.state='DISCONNECTED';item.position_trusted=false;item.stale=true;}
    updateLogicalMotor(item);
  }
}
buildLogicalMotors();


export {updateCoordinated};
