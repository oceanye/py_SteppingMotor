const {test: base, expect} = require('@playwright/test');

// Every scenario also asserts no uncaught JS error or blocked bundled script.
const test = base.extend({
  page: async ({page}, use) => {
    const errors=[];
    page.on('pageerror', e=>errors.push(e.message));
    page.on('console', msg=>{
      if(msg.type()==='error' && /Content Security Policy|Refused to (execute|load)/i.test(msg.text()))errors.push(msg.text());
    });
    await use(page);
    expect(errors, 'uncaught browser errors / CSP violations').toEqual([]);
  },
});
const card=(page,role)=>page.locator(`[data-logical-role="${role}"]`);
async function open(page){
  await page.goto('/');
  await expect(page.locator('#conn')).toHaveText('串口已连接');
}
async function controlledStatus(page, request){
  let status=await (await request.get('/api/status')).json();
  let failure=false;
  await page.route('**/api/status',route=>failure
    ? route.fulfill({status:503,json:{error:'offline test failure'}})
    : route.fulfill({json:status}));
  return {get status(){return status;},set status(value){status=value;},fail(value){failure=value;}};
}
function capturePosts(page, path){
  const requests=[];
  page.on('request',r=>{if(r.method()==='POST'&&new URL(r.url()).pathname===path)requests.push(r.postDataJSON());});
  return requests;
}

test('real server: four role units, measured position, needle and no phantom STEP/DIR',async({page})=>{
  await open(page);
  await expect(page.locator('.logical-card')).toHaveCount(4);
  for(const role of ['Mup1','Mup2']){
    await expect(card(page,role).locator('.logical-position')).toHaveText('8 mm');
    await expect(card(page,role).locator('.logical-detail')).toContainText('指令目标 16 mm');
    await expect(card(page,role).locator('.logical-detail')).toContainText('仅显示 mm 数值');
    await expect(card(page,role).locator('.linear-marker')).toBeHidden();
  }
  for(const role of ['Mr1','Mr2']){
    await expect(card(page,role).locator('.logical-position')).toHaveText('90 deg');
    await expect(card(page,role).locator('.needle')).toHaveAttribute('style',/rotate\(90deg\)/);
    await expect(card(page,role).locator('.logical-badge')).toHaveText('X42S 已到位');
  }
  await expect(page.locator('#legacySteppers')).toBeHidden();
  await expect(page.locator('[data-axis-card]')).toHaveCount(0);
  await expect(page.getByText('独立 GEAR 电机（维护选项，默认关闭）')).toBeVisible();
});

test('live updates distinguish movement, target and measured zero',async({page,request})=>{
  const state=await controlledStatus(page,request);
  const motor=state.status.logical_motors[1];
  motor.position=0;motor.target_position=180;motor.in_progress=true;
  await open(page);
  await expect(card(page,'Mr1').locator('.logical-position')).toHaveText('0 deg');
  await expect(card(page,'Mr1').locator('.logical-badge')).toHaveText('X42S 运动中');
  motor.position=25;motor.in_progress=false;
  await expect(card(page,'Mr1').locator('.needle')).toHaveAttribute('style',/rotate\(25deg\)/);
  await expect(card(page,'Mr1').locator('.logical-detail')).toContainText('指令目标 180 deg');
});

test('missing position is unknown, never target or zero',async({page,request})=>{
  const state=await controlledStatus(page,request);
  Object.assign(state.status.logical_motors[1],{position:null,position_trusted:false,target_position:180});
  await open(page);
  await expect(card(page,'Mr1').locator('.logical-position')).toHaveText('--');
  await expect(card(page,'Mr1').locator('.logical-badge')).toHaveText('X42S 反馈待确认');
  await expect(card(page,'Mr1').locator('.needle')).toHaveCSS('opacity','0.25');
});

for(const [label,code] of [['stale','X42S_STALE'],['offline','X42S_OFFLINE'],['fault','X42S_FAULT'],['estop','HARDWARE_ESTOP']]){
  test(`${label}: old position is labelled and cannot look fresh`,async({page,request})=>{
    const state=await controlledStatus(page,request);
    Object.assign(state.status.logical_motors[1],{state:code,position_trusted:false,stale:true});
    await open(page);
    await expect(card(page,'Mr1').locator('.logical-position')).toContainText('历史 / 未确认');
    await expect(card(page,'Mr1').locator('.logical-badge')).not.toHaveClass(/good/);
    await expect(card(page,'Mr1').locator('.needle')).toHaveCSS('opacity','0.25');
  });
}

test('partial snapshot and unbinding remove previous positions',async({page,request})=>{
  const state=await controlledStatus(page,request);
  await open(page);
  state.status.logical_motors=state.status.logical_motors.slice(0,1);
  await expect(card(page,'Mr2').locator('.logical-position')).toHaveText('--');
  await expect(card(page,'Mr2').locator('.logical-detail')).toContainText('本次状态缺失');
  state.status.logical_motors=[{role:'Mup1',required_mode:'linear',state:'UNBOUND',position:null,binding:null}];
  await expect(card(page,'Mup1').locator('.logical-badge')).toHaveText('未绑定');
  await expect(card(page,'Mup1').locator('.logical-position')).toHaveText('--');
});

test('disconnect and status failure disable commands, clear telemetry, then recover',async({page,request})=>{
  const state=await controlledStatus(page,request);
  await open(page);
  state.status.connected=false;
  await expect(page.locator('#conn')).toContainText('串口未连接');
  await expect(page.locator('[data-motor="enable"]').first()).toBeDisabled();
  await expect(card(page,'Mr1').locator('.needle')).toHaveCSS('opacity','0.25');
  state.fail(true);
  await expect(page.locator('#conn')).toHaveText('网页服务连接中断');
  await expect(card(page,'Mr1').locator('.logical-position')).toHaveText('--');
  await expect(page.locator('#estop')).toBeEnabled();
  state.status.connected=true;state.fail(false);
  await expect(page.locator('#conn')).toHaveText('串口已连接');
  await expect(card(page,'Mr1').locator('.logical-position')).toHaveText('90 deg');
});

test('status timeout releases polling lock instead of freezing forever',async({page,request})=>{
  const status=await (await request.get('/api/status')).json();
  let blocked=true;
  await page.route('**/api/status',async route=>{
    if(blocked){await new Promise(resolve=>setTimeout(resolve,4500));}
    await route.fulfill({json:status}).catch(()=>{});
  });
  await page.goto('/');
  await expect(page.locator('#conn')).toHaveText('网页服务连接中断');
  blocked=false;
  await expect(page.locator('#conn')).toHaveText('串口已连接');
});

test('malformed status and unexpected role do not leave stale cards or inject markup',async({page,request})=>{
  const state=await controlledStatus(page,request);
  state.status.logical_motors.push({role:'\"]<img src=x onerror=alert(1)>',position:15});
  state.status.logical_motors[1].binding_error='<img src=x onerror=alert(1)>';
  await open(page);
  await expect(card(page,'Mr1').locator('.logical-detail')).toContainText('<img');
  await expect(page.locator('#logicalMotors img')).toHaveCount(0);
  state.status={};
  await expect(page.locator('#conn')).toHaveText('网页服务连接中断');
});

test('invalid and empty GEAR target is handled locally without POST',async({page})=>{
  const posts=capturePosts(page,'/api/motor');
  await open(page);
  for(const value of ['', '3601', '-3601']){
    await page.locator('#target0').fill(value);
    await page.locator('[data-motor="target"][data-axis="0"]').click();
    await expect(page.locator('#toast')).toContainText('有效数值');
  }
  expect(posts).toEqual([]);
});

test('opted-in GEAR commands reach real adapter; retired FOC is rejected',async({page,request})=>{
  const posts=capturePosts(page,'/api/motor');
  await open(page);
  await page.locator('#target0').fill('-12.5');
  await page.locator('[data-motor="target"][data-axis="0"]').click();
  await expect(page.locator('#toast')).toHaveText('GEAR 轴 1：target');
  expect(posts[0]).toEqual({mode:'GEAR',axis:0,action:'target',target_deg:-12.5});
  for(const action of ['enable','zero','disable']){
    await page.locator(`[data-motor="${action}"][data-axis="0"]`).click();
    await expect(page.locator('#toast')).toHaveText(`GEAR 轴 1：${action}`);
  }
  await expect(page.locator('#mode0 option[value="FOC"]')).toHaveCount(0);
  const result=await request.post('/api/motor',{data:{mode:'FOC',axis:0,action:'enable'}});
  expect(result.status()).toBe(400);
  expect((await result.json()).error).toContain('FOC 已退役');
});

test('FOC status cannot restore retired controls; default-off GEAR leaves track D available',async({page,request})=>{
  const state=await controlledStatus(page,request);
  state.status.mode='FOC';
  state.status.gear_enabled=false;
  await open(page);
  await expect(page.locator('#gearSection')).toBeHidden();
  await expect(page.locator('#trackFwd')).toBeVisible();
  await expect(page.locator('[data-motor="enable"]').first()).toBeDisabled();
  await page.waitForTimeout(1100);
  await expect(page.locator('#gearSection')).toBeHidden();
});

test('invalid track PWM does not create a lease or JS error',async({page})=>{
  const posts=capturePosts(page,'/api/track');
  await open(page);
  await page.locator('#trackPwm').fill('');
  await page.locator('#trackFwd').dispatchEvent('pointerdown',{pointerId:1});
  await expect(page.locator('#toast')).toContainText('有效数值');
  await page.waitForTimeout(450);
  expect(posts).toEqual([]);
});

for(const stopEvent of ['pointerup','pointercancel','blur']){
  test(`track hold renews lease and ${stopEvent} stops renewal`,async({page})=>{
    const posts=capturePosts(page,'/api/track');
    await open(page);
    const button=page.locator('#trackFwd');
    await button.dispatchEvent('pointerdown',{pointerId:1});
    await expect.poll(()=>posts.filter(p=>p.action==='forward').length).toBeGreaterThan(1);
    if(stopEvent==='blur')await page.evaluate(()=>window.dispatchEvent(new Event('blur')));
    else await button.dispatchEvent(stopEvent,{pointerId:1});
    await expect.poll(()=>posts.some(p=>p.action==='stop')).toBeTruthy();
    const count=posts.filter(p=>p.action==='forward').length;
    await page.waitForTimeout(700);
    expect(posts.filter(p=>p.action==='forward')).toHaveLength(count);
    expect(posts.find(p=>p.action==='forward')).toEqual({action:'forward',pwm:60,lease_ms:1000});
  });
}

test('touch/mouse actual press releases the track',async({page,isMobile})=>{
  const posts=capturePosts(page,'/api/track');
  await open(page);
  const button=page.locator('#trackRev');
  await button.scrollIntoViewIfNeeded();
  if(isMobile)await button.tap();
  else{
    const rect=await button.boundingBox();
    await page.mouse.move(rect.x+rect.width/2,rect.y+rect.height/2);
    await page.mouse.down();await page.mouse.up();
  }
  await expect.poll(()=>posts.some(p=>p.action==='reverse')).toBeTruthy();
  await expect.poll(()=>posts.some(p=>p.action==='stop')).toBeTruthy();
});

test('late lease reply after release does not restart movement',async({page})=>{
  const posts=capturePosts(page,'/api/track');
  let release;
  const delayed=new Promise(resolve=>release=resolve);
  await page.route('**/api/track',async route=>{
    if(route.request().postDataJSON().action!=='stop')await delayed;
    await route.fulfill({json:{ok:true,confirmed:true}});
  });
  await open(page);
  await page.locator('#trackFwd').dispatchEvent('pointerdown',{pointerId:1});
  await expect.poll(()=>posts.length).toBe(1);
  await page.locator('#trackFwd').dispatchEvent('pointerup',{pointerId:1});
  release();
  await expect.poll(()=>posts.filter(p=>p.action==='stop').length).toBeGreaterThanOrEqual(1);
  await page.waitForTimeout(700);
  expect(posts.filter(p=>p.action==='forward')).toHaveLength(1);
  expect(posts.at(-1).action).toBe('stop');
});

test('estop requires explicit confirmation and displays failures honestly',async({page})=>{
  await open(page);
  await page.locator('#estop').click();
  await expect(page.locator('#toast')).toHaveText('停车核验已通过（不替代现场安全确认）');
  await page.route('**/api/estop',route=>route.fulfill({json:{ok:true,confirmed:false}}));
  await page.locator('#estop').click();
  await expect(page.locator('#toast')).toContainText('停车未确认');
});

test('explicit legacy UI preserves edited fields across polls and blocks unapplied move',async({page,request})=>{
  const state=await controlledStatus(page,request);
  state.status.legacy_stepdir_enabled=true;
  state.status.topology={axes:[{axis:0,label:'Archive 0',controller:'esp32',node:null,local_axis:0}]};
  state.status.stepper_axes=[{mode:'linear',pulse_per_rev:200,gear_ratio:1,lead_mm:8,position:0,position_trusted:true,travel_min:0,travel_max:100}];
  state.status.steppers=['0 mm'];
  await open(page);
  await expect(page.locator('#legacySteppers')).toBeVisible();
  await page.locator('#slead0').fill('10');
  await page.waitForTimeout(1100);
  await expect(page.locator('#slead0')).toHaveValue('10');
  await page.locator('[data-step="forward"]').click();
  await expect(page.locator('#toast')).toContainText('尚未应用的配置');
  await page.route('**/api/stepper/config',route=>route.fulfill({json:{ok:true,...route.request().postDataJSON()}}));
  await page.locator('[data-step="applycfg"]').click();
  await expect(page.locator('[data-axis-card="0"]')).not.toHaveClass(/config-dirty/);
  await page.locator('#smode0').selectOption('rotary');
  await expect(page.locator('#sunit0')).toHaveText('°');
  await expect(page.locator('#slead0')).toBeDisabled();
});

test('responsive viewport and browser zoom do not overflow horizontally',async({page})=>{
  await open(page);
  for(const width of [320,390,768,1480]){
    await page.setViewportSize({width,height:900});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth)).toBeTruthy();
  }
  await page.locator('body').evaluate(node=>node.style.zoom='1.5');
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth)).toBeTruthy();
});

test('HTTP assets and API reject obsolete writes, traversal, foreign origins and invalid input',async({request})=>{
  const root=await request.get('/');
  expect(root.headers()['content-security-policy']).toContain("script-src 'self'");
  for(const file of ['app.js','common.js','logical-motors.js','legacy-steppers.js','styles.css']){
    const asset=await request.get('/'+file);expect(asset.status()).toBe(200);
    expect(asset.headers()['x-content-type-options']).toBe('nosniff');
  }
  for(const path of ['/web_control.py','/.coordinated_bindings.json','/node_modules/@playwright/test/package.json','/%2e%2e/secrets','/api/coordinated/start'])
    expect((await request.get(path)).ok()).toBeFalsy();
  for(const [path,data] of [
    ['/api/stepper/stop',{axis:0}],
    ['/api/stepper/config',{axis:0,mode:'linear'}],
    ['/api/stepper/move',{axis:0,direction:'forward',distance_mm:1,speed_mm_s:1}],
  ]){
    const result=await request.post(path,{data});expect(result.status()).toBe(409);
    expect((await result.json()).error).toContain('STEP/DIR');
  }
  expect((await request.post('/api/motor',{data:{mode:'GEAR',axis:0,action:'target',target_deg:4000}})).status()).toBe(400);
  expect((await request.post('/api/estop',{data:{unexpected:true}})).status()).toBe(400);
  expect((await request.post('/api/estop',{data:{},headers:{Origin:'http://foreign.invalid'}})).status()).toBe(403);
});
