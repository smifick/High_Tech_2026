'use strict';
const $ = (s) => document.querySelector(s);
const escapeHTML = (value) => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let state, initialized = false, apiReady = false;
const lastFrames = new Map();
const labels = {flight:'Полёт и камеры',inventory:'Карта адресов',rover:'Ровер',settings:'Подключение'};
function notice(text, error=false) { $('#banner').textContent=text; $('#banner').className=error?'error':''; $('#banner').hidden=false; }
async function post(action, data={}) {
  if(!apiReady) throw new Error('Нет связи с локальной станцией');
  const res=await fetch('/api/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Station-Key':state.token},body:JSON.stringify(data)});
  const content=await res.text(); let result; try{result=JSON.parse(content)}catch{throw new Error(content || `HTTP ${res.status}`)}
  if(!res.ok)throw new Error(result.error || content);
  return result;
}
function task(fn) { return async event => { try { await fn(event); } catch(error) {notice(error.message,true)} }; }
async function confirmAction(title,text) {
  const dialog=$('#confirm'); $('#confirm-title').textContent=title; $('#confirm-text').textContent=text;
  dialog.returnValue='cancel'; dialog.showModal();
  return new Promise(resolve => dialog.addEventListener('close',()=>resolve(dialog.returnValue==='go'),{once:true}));
}
for (const button of document.querySelectorAll('nav button')) button.onclick=()=>{
  releaseRover();
  for(const tab of document.querySelectorAll('.tab')) tab.hidden=tab.id!=='tab-'+button.dataset.tab;
  for(const nav of document.querySelectorAll('nav button')) nav.classList.toggle('selected',nav===button);
  $('#page-title').textContent=labels[button.dataset.tab];
};
function initialize(data) {
  $('#routes').value=JSON.stringify({routes:data.routes},null,2);
  $('#drones').innerHTML=data.drones.map((d,i)=>`<article class="drone-card" data-id="${escapeHTML(d.id)}"><div class="card-head"><div><h3>${escapeHTML(d.name)}</h3><small>${escapeHTML(d.id)} · камера ${i+1}</small></div><span class="badge drone-connection">Нет связи</span></div><div class="camera"><div class="placeholder"><span class="lens">◎</span><span class="placeholder-text">Включите камеры, чтобы получить снимок</span></div><img hidden alt="Камера ${escapeHTML(d.name)}"><div class="camera-overlay"><span>CAM ${String(i+1).padStart(2,'0')} / SNAPSHOT</span><span class="image-age">НЕТ КАДРА</span></div></div><div class="camera-status">Камера выключена</div><div class="control"><p class="eyebrow">ЦЕЛЬ В ARUCO_MAP</p><div class="coords"><label>X, М<input name="x" type="number" step="0.1" min="-20" max="20" value="${i===0?'1.5':'-1.5'}" required></label><label>Y, М<input name="y" type="number" step="0.1" min="-20" max="20" value="0" required></label><label>Z, М<input name="z" type="number" step="0.1" min="0.2" max="5" value="1.7" required></label></div><div class="actions"><button class="primary navigate">Отправить цель ↗</button><button class="land">Посадка ↓</button></div><p class="command-result">Команд ещё не было</p></div></article>`).join('');
  $('#camera-settings').innerHTML=data.drones.map(d=>`<label>${escapeHTML(d.name)} · ${escapeHTML(d.id)}<input data-camera-id="${escapeHTML(d.id)}" type="url" required value="${escapeHTML(d.camera_url)}"></label>`).join('');
  for(const card of document.querySelectorAll('.drone-card')) {
    card.querySelector('.navigate').onclick=task(async()=>{
      const params={};for(const input of card.querySelectorAll('input')){if(!input.reportValidity())return;params[input.name]=Number(input.value)}
      if(!await confirmAction('Отправить цель '+card.dataset.id,`X ${params.x} · Y ${params.y} · Z ${params.z} м. Если дрон на земле, первая команда navigate запускает встроенный взлёт. Проверка препятствий и других дронов для ручной цели не выполняется.`))return;
      await post('command',{id:card.dataset.id,command:'navigate',params});notice('Команда принята сервером. Дождитесь подтверждения дрона.');
    });
    card.querySelector('.land').onclick=task(async()=>{await post('command',{id:card.dataset.id,command:'land'});notice('Посадка запрошена. При активной миссии будут посажены оба дрона после текущих команд.');});
  }
  $('#ws-address').textContent=`ws://IP_НОУТБУКА:${data.ws_port}`;
  initialized=true;
}
function render(data) {
  if(!initialized)initialize(data);
  const online=data.drones.filter(d=>d.connected).length, values=Object.values(data.states);
  $('#connected-count').innerHTML=`${online} <em>/ ${data.drones.length}</em>`;
  $('#read-count').innerHTML=`${values.length} <em>/ ${data.map.length}</em>`;
  $('#normal-count').textContent=values.filter(v=>v==='занято').length;
  $('#anomaly-count').textContent=values.filter(v=>v==='аномалия').length;
  $('#connection').textContent='Станция онлайн';$('#connection').className='badge good';
  $('#mission-badge').textContent=data.mission_active?'Выполняется':'Ожидание';
  $('#start-mission').disabled=online!==2 || data.drones.some(d=>d.busy);
  $('#stop-all').disabled=online===0 && !data.rover?.status?.connected;
  renderRover(data.rover);
  $('#inspection-status').textContent=data.inspection_status||'';
  $('#camera-mode').disabled=data.camera_mode!=='off';
  $('#cameras').textContent=data.camera_mode==='off'?'Включить камеры':'Остановить камеры';
  $('#reset-inventory').disabled=data.camera_mode!=='off';
  for(const d of data.drones){
    const card=[...document.querySelectorAll('.drone-card')].find(c=>c.dataset.id===d.id);
    const connection=card.querySelector('.drone-connection');connection.textContent=d.connected?'WS подключён':'WS не подключён';connection.className='badge drone-connection '+(d.connected?'good':'');
    card.querySelector('.navigate').disabled=!d.connected||d.busy;
    card.querySelector('.land').disabled=!d.connected;
    const info=d.camera, age=info.received_at?Math.max(0,Date.now()/1000-info.received_at):null;
    card.querySelector('.image-age').textContent=age===null?'НЕТ КАДРА':`${Math.round(age)} С НАЗАД`;
    card.querySelector('.camera-status').textContent=info.error?`Ошибка камеры: ${info.error}`:data.camera_mode==='off'?'Камера выключена · показан последний снимок':info.received_at?`Снимок #${info.frame} · ${info.cycle_ms} мс · ${info.detections.length} детекций`:'Ожидание первого снимка…';
    const img=card.querySelector('img');
    if(info.frame && lastFrames.get(d.id)!==info.frame){
      img.onload=()=>{img.hidden=false;card.querySelector('.placeholder').hidden=true};
      img.onerror=()=>{img.hidden=true;card.querySelector('.placeholder').hidden=false;card.querySelector('.placeholder-text').textContent='Снимок недоступен'};
      img.src='/api/image?id='+encodeURIComponent(d.id)+'&mode=cv&v='+info.frame;lastFrames.set(d.id,info.frame);
    }
    if(!info.frame)card.querySelector('.placeholder-text').textContent=info.error|| (data.camera_mode==='off'?'Включите камеры, чтобы получить снимок':'Подключение к камере…');
    const p=d.progress;
    card.querySelector('.command-result').textContent=d.landing_queued?'Посадка запрошена · после текущей команды':p?`${p.command}: ${p.status==='done'?'подтверждена':p.status==='error'?'ошибка':'ожидается результат'}`:'Команд ещё не было';
  }
  $('#events').innerHTML=data.events.map(e=>`<div class="event ${escapeHTML(e.level)}"><time>${new Date(e.time*1000).toLocaleTimeString('ru-RU')}</time><span>${escapeHTML(e.text)}</span></div>`).join('');
  const colors={'пусто':'empty','занято':'occupied','аномалия':'anomaly'};
  $('#field-map').innerHTML=[...data.map].sort((a,b)=>b.y-a.y||a.x-b.x).map(m=>`<div class="marker ${colors[data.states[m.id]]||''}" title="X ${m.x}, Y ${m.y}"><b>${m.id}</b><small>${escapeHTML(data.states[m.id]||'не считано')}</small></div>`).join('');
  $('#address-list').innerHTML=Object.entries(data.states).map(([key,value])=>`<div>${escapeHTML(key)}: ${escapeHTML(value)}</div>`).join('')||'<p class="muted">Включите камеры с распознаванием, чтобы начать обход.</p>';
  $('#unmatched').innerHTML=data.drones.map(d=>`<p class="muted">${escapeHTML(d.name)}: ${(data.unmatched[d.id]||[]).length} без метки</p>`).join('');
}
$('#stop-all').onclick=task(async()=>{await post('stop');releaseRover();notice('Остановка ровера запрошена. Дроны завершат текущую команду и выполнят посадку.');});
$('#cameras').onclick=task(async()=>{await post('cameras',{mode:state.camera_mode==='off'?$('#camera-mode').value:'off'});notice('Запрос изменения режима камер принят.');});
function routeInput(){const data=JSON.parse($('#routes').value);if(!data.routes)throw new Error('JSON должен содержать объект routes');return data.routes}
$('#validate-routes').onclick=task(async()=>{await post('routes',{routes:routeInput()});notice('Маршруты корректны; горизонтальный зазор между заданными участками не менее 0,75 м.');});
$('#start-mission').onclick=task(async()=>{const routes=routeInput();await post('routes',{routes});if(!await confirmAction('Запустить маршруты двух дронов?',Object.entries(routes).map(([id,r])=>`${id}: ${r.points.length} точек, высота ${r.height??1.7} м`).join('. ')+'. После посадки обоих дронов ровер автоматически проверит адреса с аномалиями и вернётся в гараж. Команды начнут выполняться сразу.'))return;await post('mission',{routes});notice('Маршруты запущены. Следите за подтверждениями и состоянием дронов.');});
$('#route-file').onchange=task(async e=>{const file=e.target.files[0];if(!file)return;if(file.size>128*1024)throw new Error('Файл слишком большой');const data=JSON.parse(await file.text());if(!data.routes)throw new Error('Нет объекта routes');$('#routes').value=JSON.stringify(data,null,2);notice('Маршруты загружены в редактор. Полёт не запущен.');});
$('#settings-form').onsubmit=task(async e=>{e.preventDefault();const cameras={};for(const input of document.querySelectorAll('[data-camera-id]'))cameras[input.dataset.cameraId]=input.value;await post('config',{cameras});notice('Адреса камер применены для текущего запуска.');});
$('#reset-inventory').onclick=task(async()=>{if(!await confirmAction('Начать новый обход?','Накопленные состояния адресов текущей сессии будут очищены. Полёт не запускается.'))return;await post('reset');notice('Результаты очищены. Можно включить камеры.');});
async function poll(){try{const response=await fetch('/api/state');if(!response.ok)throw new Error('HTTP '+response.status);state=await response.json();apiReady=true;render(state)}catch(error){apiReady=false;releaseRover();for(const b of document.querySelectorAll('[data-drive]'))b.disabled=true;$('#connection').textContent='Станция недоступна';$('#connection').className='badge bad';for(const button of document.querySelectorAll('.navigate,.land,#start-mission,#stop-all'))button.disabled=true}finally{setTimeout(poll,1000)}}

let roverHold=null;
function releaseRover(){
  const hold=roverHold;if(!hold)return;
  roverHold=null;hold.released=true;clearTimeout(hold.timer);
  hold.button.classList.remove('held');$('#rover-hold-status').textContent='Управление не удерживается';
  if(hold.lease)roverStopping=post('rover',{operation:'stop',lease:hold.lease}).catch(e=>notice(e.message,true));
}
async function pulseRover(hold){
  if(hold!==roverHold)return;
  try{await post('rover',{operation:'pulse',lease:hold.lease});
    if(hold===roverHold)hold.timer=setTimeout(()=>pulseRover(hold),40);
  }catch(e){if(roverHold===hold)releaseRover();notice(e.message,true)}
}
let roverStarting=false;
let roverStopping=Promise.resolve();
async function startRover(button, key=null){
  if(roverHold||roverStarting||button.disabled||!apiReady)return;
  if(!$('#rover-speed').reportValidity()||!$('#rover-turn-speed').reportValidity())return;
  const hold={button,key,released:false};roverHold=hold;roverStarting=true;
  button.classList.add('held');$('#rover-hold-status').textContent='Управление удерживается';
  try{
    await roverStopping;
    if(hold.released)return;
    const result=await post('rover',{operation:'begin',direction:button.dataset.drive,speed:Number($('#rover-speed').value),turn_speed:Number($('#rover-turn-speed').value)});
    hold.lease=result.lease;
    if(hold.released||hold!==roverHold)await post('rover',{operation:'stop',lease:hold.lease});
    else hold.timer=setTimeout(()=>pulseRover(hold),40);
  }catch(error){if(roverHold===hold)releaseRover();notice(error.message,true)}
  finally{roverStarting=false}
}
for(const button of document.querySelectorAll('[data-drive]')){
  button.onpointerdown=e=>{
    if(e.button!==0||roverHold||roverStarting||button.disabled)return;
    e.preventDefault();button.setPointerCapture(e.pointerId);startRover(button);
  };
  for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,()=>{
    if(roverHold?.button===button && roverHold.key===null)releaseRover();
  });
  button.oncontextmenu=e=>e.preventDefault();
}
const roverKeys={KeyW:'forward',KeyS:'backward',KeyA:'left',KeyD:'right',KeyQ:'turn_right',KeyE:'turn_left'};
function roverTyping(target){return target?.isContentEditable || Boolean(target?.closest?.('input,textarea,select,[contenteditable],dialog[open]'))}
document.addEventListener('keydown',e=>{
  if($('#tab-rover').hidden||document.hidden||roverTyping(e.target)||document.querySelector('dialog[open]'))return;
  if(e.code==='Space'){e.preventDefault();releaseRover();if(!e.repeat)post('rover',{operation:'stop'}).catch(error=>notice(error.message,true));return}
  const direction=roverKeys[e.code];if(!direction||e.ctrlKey||e.altKey||e.metaKey)return;
  e.preventDefault();if(e.repeat)return;
  startRover(document.querySelector(`[data-drive="${direction}"]`),e.code);
});
document.addEventListener('keyup',e=>{if(roverHold?.key===e.code){e.preventDefault();releaseRover()}});
document.addEventListener('focusin',e=>{if(roverTyping(e.target))releaseRover()});
window.addEventListener('blur',releaseRover);
window.addEventListener('pagehide',releaseRover);
document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseRover()});
$('#rover-stop').onclick=task(async()=>{releaseRover();await post('rover',{operation:'stop'});notice('Ровер: остановка подтверждена');});
$('#rover-camera').onclick=task(async()=>{await post('rover-camera',{mode:state.rover.camera_mode==='off'?$('#rover-mode').value:'off'});});
function renderRover(rover){
  const connected=Boolean(rover?.status?.connected),info=rover?.camera||{},drive=rover?.status?.drive||{};
  $('#rover-status').textContent=connected?(rover.status.degraded?'Связь нестабильна':'Ровер подключён'):'Нет связи с ровером';$('#rover-status').className='badge '+(connected?'good':'bad');
  for(const b of document.querySelectorAll('[data-drive]'))b.disabled=state.mission_active||!connected||(['left','right'].includes(b.dataset.drive)&&drive.drive_type!=='mecanum');
  if(!connected)releaseRover();
  $('#rover-camera').disabled=!rover?.available;
  $('#rover-camera').textContent=rover?.camera_mode==='off'?'Включить камеру':'Остановить камеру';
  $('#rover-mode').disabled=rover?.camera_mode!=='off';
  $('#rover-drive-status').textContent=connected?`${rover.status.degraded?'Задержка опроса · ':''}${drive.drive_type} · watchdog ${drive.timeout_sec} с · ограничение панели 0,2 м/с`:rover?.status?.error||'Адаптер не подключён';
  const age=info.received_at?Math.round(Date.now()/1000-info.received_at):null;
  $('#rover-camera-status').textContent=info.error|| (rover?.camera_mode==='off'?'Камера выключена · показан последний снимок':age===null?'Ожидание снимка…':`Снимок #${info.frame} · ${age} с назад · ${info.cycle_ms} мс`);
  if(info.frame&&lastFrames.get('rover')!==info.frame){const img=$('#rover-image');img.onload=()=>{img.hidden=false;$('#rover-placeholder').hidden=true};img.src='/api/image?id=rover&mode=cv&v='+info.frame;lastFrames.set('rover',info.frame)}
  $('#rover-detections').innerHTML=(info.detections||[]).map(d=>`<p><span class="badge ${d.anomaly?'bad':'good'}">${d.anomaly?'Аномалия':'Коробка'}</span> ${Math.round(d.confidence*100)}%</p>`).join('')||'<p class="muted">Нет детекций в последнем снимке</p>';
}
poll();
