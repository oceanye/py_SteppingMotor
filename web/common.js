const toast = document.getElementById('toast');
let toastTimer;
function notice(text,bad=false,duration=2600){toast.textContent=text;toast.className='toast show'+(bad?' bad':'');clearTimeout(toastTimer);toastTimer=setTimeout(()=>toast.className='toast',duration);}
async function api(path, body){
  const controller=new AbortController();
  const timer=setTimeout(()=>controller.abort(),4000);
  const opt={headers:{},signal:controller.signal,cache:'no-store'};
  if(body!==undefined){opt.method='POST';opt.headers['Content-Type']='application/json';opt.body=JSON.stringify(body);}
  try {
    const res=await fetch(path,opt);
    const data=await res.json();
    if(!res.ok || data?.ok===false) throw new Error(data?.error || `请求失败 (${res.status})`);
    if(!data || typeof data!=='object' || Array.isArray(data)) throw new Error('服务响应格式错误');
    return data;
  } finally { clearTimeout(timer); }
}
function num(id,min,max){const raw=document.getElementById(id).value.trim(),v=Number(raw);if(!raw||!Number.isFinite(v)||v<min||v>max)throw new Error(`请输入 ${min}–${max} 的有效数值`);return v;}
function act(job,ok,onSuccess){return Promise.resolve().then(job).then(data=>{if(onSuccess)onSuccess(data);const message=typeof ok==='function'?ok(data):ok;if(message)notice(message);return data;}).catch(e=>notice(e.message,true,4200));}

function formatNumber(value,digits=3){
  if(value===null||value===undefined||value==='')return '--';
  const number=Number(value);
  return Number.isFinite(number)?number.toLocaleString('zh-CN',{maximumFractionDigits:digits}):'--';
}

export {notice, api, num, act, formatNumber};
