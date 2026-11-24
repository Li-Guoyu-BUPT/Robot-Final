"use strict";
(() => {
  try{history.scrollRestoration='manual'}catch(e){}
  const byId=id=>document.getElementById(id);
  const page=()=>['manual','gallery'].includes(location.hash.slice(1))?location.hash.slice(1):'home';
  let active=page(), data=null, selected=null, previewUrl=null, signature='', travelMode='manual';
  const bundledMaps=[{id:'photo_lab_batch3_dense_20261120',time:'2026-11-20T22:56:06',points:912972,optimized:true,session:'photo_reconstruction',title:'实验室照片重建 · 三批联合优化',preview:'/assets/maps/lab_batch3/稠密地图预览.html',source:'手机照片离线重建 · 三批联合优化',note:'172张照片输入，167张有效视图；单目尺度未标定。'}];
  try{selected=sessionStorage.getItem('roban.selectedMap');travelMode=sessionStorage.getItem('roban.rescanTravelMode')==='auto'?'auto':'manual'}catch(e){}
  function selectMap(id){
    selected=id;try{sessionStorage.setItem('roban.selectedMap',id)}catch(e){}
    renderDetail();
    if(innerWidth<800)byId('map-detail-content').scrollIntoView({block:'start'});
  }
  function route(){
    const next=page();
    if(next!==active&&(next==='manual'||active==='manual')){
      if(window.manualConsoleLeave)window.manualConsoleLeave();
      location.reload();return;
    }
    active=next;
    ['home','manual','gallery'].forEach(name=>{byId(name+'-page').hidden=name!==active});
    document.querySelectorAll('.platform-nav a').forEach(a=>{
      if(a.hash==='#'+active)a.setAttribute('aria-current','page');else a.removeAttribute('aria-current');
    });
    document.title=({home:'工作台',manual:'人工建图',gallery:'图库'})[active]+' · ROBAN';
    window.scrollTo(0,0);
  }
  window.addEventListener('hashchange',route);route();
  if(active==='manual'){
    try{
      const task=JSON.parse(sessionStorage.getItem('roban.rescanTask')||'null');
      if(task?.mode==='manual'){
        byId('rescan-context').hidden=false;
        byId('rescan-context-text').textContent='人工前往补扫区域 · 所选地图：'+task.mapTitle+'。地图对应与目标点识别待接入，当前不会自动恢复该地图；请在下方存档区确认需要使用的地图。';
      }
    }catch(e){}
    startManualConsole();return;
  }
  const number=n=>Number(n||0).toLocaleString('zh-CN');
  const date=s=>(s||'').replace('T',' ');
  const kind=a=>a.optimized?'优化地图':a.session.startsWith('web_replay')?'回放地图':'人工建图';
  function title(a){return a.title||kind(a)+' · '+date(a.time).slice(5,16)}
  function cell(row,value){const td=document.createElement('td');td.textContent=value;row.append(td);return td}
  function renderDetail(){
    const a=(data?.archives||[]).find(a=>a.id===selected);
    byId('map-detail-empty').hidden=!!a;byId('map-detail-content').hidden=!a;
    if(!a)return;
    byId('detail-title').textContent=title(a);
    byId('detail-time').textContent=date(a.time);
    byId('detail-source').textContent=kind(a);
    byId('detail-points').textContent=number(a.points)+' 点';
    const preview=byId('detail-preview'); if(preview){preview.hidden=!a.preview;preview.href=a.preview||'#';}
    const note=byId('detail-note'); if(note)note.textContent=a.note||'三维存档预览将在后续接入。原始地图可在人工建图页面的存档列表中恢复。';
    byId('detail-id').textContent=a.id;
    byId('selection-status').textContent='已选定地图；机器人定位与自主补扫尚未接通。';
    renderTravelMode();
    document.querySelectorAll('[data-map-id]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.mapId===selected)));
  }
  function renderTravelMode(){
    const automatic=travelMode==='auto';
    byId('travel-manual').checked=!automatic;byId('travel-auto').checked=automatic;
    byId('travel-mode-help').textContent=automatic?
      '后续由机器人根据所选地图与实时观测规划、避障并前往补扫位置；定位失效或路径受阻时暂停，可切回人工。当前自主执行未接通。':
      '进入现有操控台，由你用键盘或鼠标单步前往补扫区域。地图定位与自动识别补扫点仍待接入。';
    byId('travel-action').textContent=automatic?'自主前往待接入':'进入人工操控';
    byId('travel-action').disabled=automatic||!selected;
    byId('auto-prerequisites').hidden=!automatic;
    try{sessionStorage.setItem('roban.rescanTravelMode',travelMode)}catch(e){}
  }
  function renderLibrary(){
    const q=byId('map-search').value.trim().toLowerCase(),type=byId('map-filter').value;
    const all=data?.archives||[];
    const filtered=all.filter(a=>(type==='all'||(type==='optimized'&&a.optimized)||(type==='manual'&&!a.optimized&&!a.session.startsWith('web_replay')))&&
      (title(a)+' '+a.id).toLowerCase().includes(q));
    byId('library-count').textContent=all.length?'当前服务提供 '+all.length+' 个近期存档':'尚无地图存档';
    byId('map-table-body').replaceChildren();
    byId('library-empty').hidden=filtered.length>0;
    byId('library-empty').textContent=all.length?'没有符合筛选条件的地图。试试其他日期或清除筛选。':'还没有地图。可以先进入人工建图，完成采集后点击“地图存档”。';
    for(const a of filtered){
      const tr=document.createElement('tr');
      const td=cell(tr,'');const b=document.createElement('button');b.type='button';b.className='map-select';
      b.textContent=title(a);b.dataset.mapId=a.id;b.setAttribute('aria-pressed',String(a.id===selected));
      b.onclick=()=>selectMap(a.id);td.append(b);
      cell(tr,date(a.time));cell(tr,number(a.points));cell(tr,kind(a));
      byId('map-table-body').append(tr);
    }
    renderDetail();
  }
  function renderHome(){
    const connected=data?.connection==='connected';
    byId('platform-connection').textContent=connected?'机器人已连接':'机器人未连接';
    byId('platform-connection').classList.toggle('is-connected',connected);
    byId('device-summary').textContent=connected?'ROBAN · '+data.host:'可浏览图库；人工建图需要连接机器人。';
    byId('home-camera').hidden=!connected||!data.capture?.frames;
    byId('home-camera-empty').hidden=connected&&!!data.capture?.frames;
    if(connected&&data.capture?.frames)byId('home-camera').src='/api/camera.jpg?t='+data.capture.frames;
    byId('camera-caption').textContent=connected?(data.capture.stale?'当前画面已过期':`相机接收 ${data.capture.fps.toFixed(1)} FPS · ${Math.round(data.capture.age_ms)} ms 前收到`):'等待相机连接';
    byId('recent-maps').replaceChildren();
    for(const a of (data.archives||[]).slice(0,4)){
      const li=document.createElement('li');const b=document.createElement('button');b.className='recent-map';
      const name=document.createElement('span'),meta=document.createElement('span');name.textContent=title(a);meta.textContent=number(a.points)+' 点';
      b.append(name,meta);b.onclick=()=>{location.hash='gallery';selectMap(a.id);renderLibrary();};li.append(b);byId('recent-maps').append(li);
    }
    byId('recent-empty').hidden=!!data.archives?.length;
  }
  async function refresh(){
    try{
      const response=await fetch('/api/status',{cache:'no-store'});if(!response.ok)throw Error('服务暂时不可用');
      data=await response.json();data.archives=[...bundledMaps,...(data.archives||[]).filter(a=>!bundledMaps.some(b=>b.id===a.id))];byId('platform-error').hidden=true;renderHome();
      const next=JSON.stringify(data.archives||[]);
      if(signature!==next){signature=next;renderLibrary();}
    }catch(e){
      data={archives:bundledMaps,connection:'disconnected'};renderHome();renderLibrary();
      byId('platform-error').hidden=false;byId('platform-error').textContent='本机服务连接中断。请检查控制台运行窗口，再点击“刷新地图”。';
      byId('platform-connection').textContent='服务未连接';byId('platform-connection').classList.remove('is-connected');
      byId('home-camera').hidden=true;byId('home-camera-empty').hidden=false;
    }
  }
  byId('map-search').addEventListener('input',renderLibrary);
  document.querySelectorAll('input[name="rescan-travel-mode"]').forEach(r=>r.addEventListener('change',()=>{travelMode=r.value;renderTravelMode()}));
  byId('travel-action').onclick=()=>{
    const a=(data?.archives||[]).find(a=>a.id===selected);
    if(!a||travelMode!=='manual')return;
    try{sessionStorage.setItem('roban.rescanTask',JSON.stringify({mapId:a.id,mapTitle:title(a),mode:'manual',target:null,status:'operator_control'}))}catch(e){}
    location.hash='manual';
  };
  byId('map-filter').addEventListener('change',renderLibrary);
  byId('refresh-library').onclick=refresh;
  byId('clear-filter').onclick=()=>{byId('map-search').value='';byId('map-filter').value='all';renderLibrary()};
  byId('open-import').onclick=()=>{byId('video-import').hidden=!byId('video-import').hidden;byId('open-import').setAttribute('aria-expanded',String(!byId('video-import').hidden))};
  byId('video-input').onchange=()=>{
    const file=byId('video-input').files[0];if(previewUrl){URL.revokeObjectURL(previewUrl);previewUrl=null;}
    byId('video-preview').removeAttribute('src');byId('video-preview').hidden=true;
    if(!file){byId('video-info').textContent='选择MP4视频，在此预览。视频尚不会上传或重建。';return;}
    if(!/\.mp4$/i.test(file.name)){byId('video-info').textContent='请选择 .mp4 文件。';byId('video-input').value='';return;}
    previewUrl=URL.createObjectURL(file);byId('video-preview').src=previewUrl;byId('video-preview').hidden=false;
    byId('video-info').textContent=file.name+' · '+(file.size/1024/1024).toFixed(1)+' MB · 仅本地预览，尚未上传。';
  };
  byId('video-preview').onerror=()=>{byId('video-info').textContent='浏览器无法预览该视频编码。文件未上传，可换用H.264编码的MP4。'};
  window.addEventListener('pagehide',()=>{if(previewUrl)URL.revokeObjectURL(previewUrl)});
  refresh();setInterval(refresh,3000);
})();
