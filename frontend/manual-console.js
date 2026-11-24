"use strict";
const $=id=>document.getElementById(id);
const client=crypto.randomUUID();
let state=null,ended=false,localBusy=false,lastCamera=-1,lastRevision=-1,statusBusy=false,cloudBusy=false;
let feedback=null;
let mapBusy=false,lastGeneration=null,archiveSignature="";
let lastMeshRevision=-1,meshBusy=false;
const pressed=new Set();
let requireRelease=false;
const token=window.BOOT.token;
async function post(route,data={}){
  const response=await fetch(route,{method:"POST",headers:{"Content-Type":"application/json","X-Control-Token":token},body:JSON.stringify(data)});
  const result=await response.json();
  if(!response.ok)throw new Error(result.error||"请求失败");
  return result;
}
function notice(text,type=""){const el=$("notice");el.textContent=text;el.className=type}
function feedbackNotice(text,type=""){feedback={text,type,until:Date.now()+8000};notice(text,type)}
function phaseLabel(c){return ({blocked:"步态已停用",disarmed:"未启用",idle:"待命",pending:"等待本体",executing:"单步执行中",preparing:"正在准备",dispatching:"正在提交"})[c.phase]||c.phase}
function canStep(){return state&&state.mode==="live"&&state.motion_allowed!==false&&state.control.command_steps&&!state.capture.stale&&state.control.armed&&state.control.phase==="idle"&&!state.control.uncertain&&!localBusy&&!mapBusy&&!state.map_busy&&!ended}
function render(s){
  state=s;
  if(lastGeneration!==s.map_generation){viewer.clear();lastRevision=-1;lastMeshRevision=-1;lastGeneration=s.map_generation}
  $("mode").textContent=s.mode==="live"?(s.motion_allowed===false?"真机仅采集 · 步态停用":"真机连接"):"历史回放 · 运动禁用";
  document.querySelector('.control-note').textContent=s.motion_allowed===false?"现场已报告跌倒，准备控制和所有方向键已停用。当前只做传感器采集、建图和离线排查。":"每次一个动作：前进 5 cm / 后退 4 cm / 侧移 3 cm / 转向 10°。设定值需现场标定；长按不连走。";
  document.querySelector('.fine-print').textContent=s.motion_allowed===false&&s.mode==='live'?"当前只连接传感器，网页没有步态控制通道。软件停用不等于舵机断电，请由现场人员按标准流程处理本体。":"F8 可能使舵机进入准备姿态。切离页面禁止新增动作；已下发的一步按本体反馈收尾。停止按钮请求受控停止。";
  $("mode").className="badge "+(s.mode==="live"?"live":"replay");
  $("device").textContent=s.host;
  $("frames").textContent=s.capture.frames.toLocaleString();
  $("fps").textContent=s.capture.fps.toFixed(1);
  $("integrated").textContent=s.map.integrated.toLocaleString();
  $("points").textContent=s.map.points.toLocaleString();
  $("map-state").textContent=s.map_busy?"正在处理地图":s.map.tracking==="lost"?"跟踪丢失，融合冻结":s.map.tracking==="waiting"?"等待深度帧":s.map.paused?"融合暂停，跟踪继续":s.capture.source_done?"回放完成，地图保留":s.map.motion_active?"行走中跟踪，站稳后融合":s.map.capture_pending===false?"本次关键帧已采集":s.map.post_motion_wait_s>0?"动作后等待4秒":s.map.fusion_blocked?"等待稳定或模型对齐":"持续建图中";
  $("coverage").textContent=s.map.fov_h==null?"等待相机深度与视场信息":"视场 "+s.map.fov_h.toFixed(0)+"° × "+s.map.fov_v.toFixed(0)+"° · 当前画面可用深度 "+Math.round(s.map.usable_depth_ratio*100)+"%（0.25–"+s.map.depth_max+" m），不代表场景覆盖率。";
  $("quality").textContent=s.map.overlap==null?"配准等待中":"配准重合 "+Math.round(s.map.overlap*100)+"% · 深度残差 "+(s.map.depth_rmse_mm==null?"—":s.map.depth_rmse_mm.toFixed(1))+" mm";
  $("camera-state").textContent=s.capture.stale?(s.capture.source_done?"回放结束":"画面过期"):Math.round(s.capture.age_ms)+" ms 前收到";
  $("camera-stale").hidden=!s.capture.stale;
  $("control-state").textContent=s.mode==="replay"?"回放禁用":phaseLabel(s.control);
  $("control-state").style.color=s.control.uncertain||s.motion_allowed===false?"#a82732":"";
  $("session-path").textContent=s.session;
  $("record-state").textContent=s.mode==="live"?"持续保存原始 RGB-D":"使用已保存的真实 RGB-D";
  $("map-time").textContent=s.map.revision?"地图更新 "+s.map.revision:"等待更新";
  $("map-empty").hidden=s.map.display_points>0;
  $("pause").textContent=s.map.paused?"继续融合":"暂停融合";
  $("save").disabled=!s.map.points||ended||mapBusy||s.map_busy;
  const busy=["preparing","dispatching","pending","executing"].includes(s.control.phase)||localBusy||mapBusy||s.map_busy;
  $("clear").disabled=ended||busy||!s.map.integrated;
  $("restore").disabled=ended||busy||!$("archives").value;
  $("pause").disabled=ended||mapBusy||s.map_busy;
  $("optimize").disabled=ended||busy||s.map.integrated<3;
  $("surface").disabled=!s.map.triangles;
  const opt=s.optimization||{status:"idle"};
  $("optimization-message").hidden=opt.status==="idle";
  const stages={archive:"存档",load:"读取稳定视角",register:"多视角配准",fuse:"按优化位姿重新融合",done:"优化完成"};
  $("optimization-message").textContent=opt.status==="running"?(stages[opt.stage]||"优化中")+" "+(opt.current||0)+" / "+(opt.total||"—")+"；相机原始记录继续。":opt.status==="failed"?"优化未通过："+opt.error+"。原图保留，融合暂停。":opt.status==="done"?"优化结果已载入，内部特征残差 "+opt.old_residual_mm.toFixed(1)+" → "+opt.new_residual_mm.toFixed(1)+" mm；检查后点击“继续融合”。":"";
  const archiveKey=JSON.stringify(s.archives||[]);
  if(archiveKey!==archiveSignature){
    const selected=$("archives").value;$("archives").replaceChildren();
    (s.archives||[]).forEach(a=>{const o=document.createElement("option");o.value=a.id;o.textContent=a.time.replace("T"," ")+" · "+a.points.toLocaleString()+" 点 · "+(a.optimized?"优化":a.session.startsWith("web_replay")?"回放":"真机");$("archives").append(o)});
    if(!$("archives").options.length)$("archives").append(new Option("暂无可恢复存档",""));
    if([...$("archives").options].some(o=>o.value===selected))$("archives").value=selected;
    archiveSignature=archiveKey;$("restore").disabled=ended||busy||!$("archives").value;
  }
  $("arm").disabled=s.mode!=="live"||s.motion_allowed===false||!s.control.command_steps||s.capture.stale||s.control.uncertain||s.control.stopping||busy||s.control.armed||ended;
  $("arm").firstChild.textContent=s.motion_allowed===false&&s.mode==='live'?"步态已停用 ":s.control.armed?"控制已准备 ":"准备控制 ";
  $("stop").disabled=s.mode!=="live"||s.motion_allowed===false||s.connection!=="connected"||ended;
  document.querySelectorAll("[data-key]").forEach(b=>b.disabled=!canStep());
  const msg=s.motion_allowed===false&&s.mode==='live'?"步态故障锁定已生效，重启页面也不会恢复移动。":s.mode==="replay"?"历史回放模式，所有机器人动作均已禁用。":s.control.uncertain?"本体停止或状态未确认。停止操作并现场检查。":busy?"当前动作处理中，新的按键不会排队。":s.control.armed?"单击方向键或按钮执行一步，长按不重复。":"先准备控制；按键不会自动启用机器人。";
  $("step-message").textContent=msg;
  if(s.motion_allowed===false&&s.mode==='live')notice("步态已停用："+(s.motion_block_reason||"跌倒排查中")+"。当前仅采集和建图，暂无步态控制连接。","error");
  else if(s.control.uncertain)notice(msg,"error");
  else if(s.map.tracking==="lost")notice("跟踪丢失，融合冻结。可单步回到有重叠的视角，或清除当前地图重新采集；原始记录继续。","warning");
  else if(feedback&&Date.now()<feedback.until)notice(feedback.text,feedback.type);
  else if(s.mode==="replay")notice(s.capture.source_done?"真实历史序列已回放完毕。地图保留，可旋转、缩放和保存；机器人控制始终禁用。":"正在回放真实 RGB-D 验证页面，机器人控制禁用。","warning");
  else if(s.capture.stale)notice("相机画面已过期，禁止新增动作。请检查采集连接。","warning");
  else if(!s.map.paused&&s.map.fusion_blocked&&!s.map.motion_active)notice(s.map.fusion_blocked,"warning");
  else notice("相机与建图持续运行。每次操作只提交一步，等待完成后再继续。");
  $("events").replaceChildren();
  s.events.slice(-6).reverse().forEach(e=>{
    const li=document.createElement("li"),t=document.createElement("time"),text=document.createElement("span");
    t.textContent=new Date(e.pc_wall*1000).toLocaleTimeString("zh-CN",{hour12:false});
    text.textContent=e.text;
    li.append(t,text);$("events").append(li);
  });
  if(s.capture.frames!==lastCamera&&s.capture.frames){
    lastCamera=s.capture.frames;
    const image=$("camera");
    image.onload=()=>{image.hidden=false;$("camera-empty").hidden=true};
    image.src="/api/camera.jpg?frame="+s.capture.frames;
  }
  viewer.setTrajectory(s.map.trajectory||[]);
  if(s.map.revision!==lastRevision&&!cloudBusy)updateCloud(s.map.revision);
  if(s.map.mesh_revision!=null&&s.map.mesh_revision!==lastMeshRevision&&!meshBusy)updateMesh(s.map.mesh_revision);
}
async function updateMesh(revision){
  meshBusy=true;
  try{
    const response=await fetch("/api/mesh.bin?revision="+revision);
    if(!response.ok)throw new Error("表面数据未就绪");
    const buffer=await response.arrayBuffer();
    if(buffer.byteLength>=8){
      const head=new DataView(buffer),vertices=head.getUint32(0,true),indices=head.getUint32(4,true);
      if(vertices>2000000||indices>12000000||buffer.byteLength!==8+vertices*24+indices*4)throw new Error("表面数据格式错误");
      viewer.setMesh(new Float32Array(buffer,8,vertices*6),new Uint32Array(buffer,8+vertices*24,indices));
    }
    lastMeshRevision=Number(response.headers.get("X-Map-Revision")||revision);
  }catch(e){console.warn(e)}finally{meshBusy=false}
}
async function updateCloud(revision){
  cloudBusy=true;
  try{
    const response=await fetch("/api/map.bin?revision="+revision);
    const buffer=await response.arrayBuffer();
    if(!buffer.byteLength){viewer.clear();lastRevision=Number(response.headers.get("X-Map-Revision")||revision)}
    if(buffer.byteLength&&buffer.byteLength%24===0){
      viewer.setPoints(new Float32Array(buffer));lastRevision=Number(response.headers.get("X-Map-Revision")||revision);
    }
  }catch(e){console.warn(e)}finally{cloudBusy=false}
}
async function poll(){
  if(statusBusy||ended)return;
  statusBusy=true;
  try{
    const r=await fetch("/api/status");if(!r.ok)throw new Error("服务未响应");
    render(await r.json());
  }catch(e){
    notice("本机服务连接中断。禁止新增动作，请检查运行窗口。","error");
    document.querySelectorAll("[data-key],#arm").forEach(b=>b.disabled=true);
    state=null;
  }finally{statusBusy=false}
}
async function heartbeat(){
  if(ended)return;
  try{await post("/api/heartbeat",{client,focused:document.hasFocus()&&!document.hidden})}catch(e){}
}
async function action(op,key){
  if(ended)return;
  if(op==="step"&&!canStep()){$("step-message").textContent="当前不能执行，本次按键未排队。";return}
  if(op==="arm"&&$("arm").disabled)return;
  localBusy=true;
  try{
    await heartbeat();
    const r=await post("/api/action",{client,op,key,command_id:crypto.randomUUID()});
    if(!r.accepted)throw new Error(r.error||"动作未被接收");
    if(op==="step")$("step-message").textContent="已提交一步，正在等待本体反馈。";
  }catch(e){feedbackNotice(e.message,"error");$("step-message").textContent=e.message}
  finally{localBusy=false;poll()}
}
document.querySelectorAll("[data-key]").forEach(b=>b.addEventListener("click",()=>action("step",b.dataset.key)));
$("arm").addEventListener("click",()=>action("arm"));
$("stop").addEventListener("click",()=>action("stop"));
$("fit").addEventListener("click",()=>viewer.fit());
function showSurface(on){viewer.surface=on;$("surface").setAttribute("aria-pressed",String(on));$("point-mode").setAttribute("aria-pressed",String(!on))}
$("surface").addEventListener("click",()=>showSurface(true));
$("point-mode").addEventListener("click",()=>showSurface(false));
$("optimize").addEventListener("click",async()=>{
  if(mapBusy)return;
  try{await post("/api/map/optimize");feedbackNotice("多视角优化已开始。原图先存档，相机记录继续。","warning");poll()}
  catch(e){feedbackNotice(e.message,"error")}
});
$("pause").addEventListener("click",async()=>{try{await post("/api/mapping",{paused:!state.map.paused});poll()}catch(e){notice(e.message,"error")}});
$("save").addEventListener("click",async()=>{
  if(mapBusy)return;mapBusy=true;$("save").disabled=true;$("save").textContent="正在存档";
  try{const r=await post("/api/save");feedbackNotice("地图已保存并读回验证："+r.path)}catch(e){feedbackNotice(e.message,"error")}
  finally{mapBusy=false;$("save").textContent="地图存档";poll()}
});
async function changeMap(restore){
  if(mapBusy||!state)return;
  mapBusy=true;feedbackNotice(restore?"正在恢复存档；相机原始记录继续。":"正在自动存档并清除地图；原始记录保留。","warning");
  try{
    await post(restore?"/api/map/restore":"/api/map/clear",restore?{archive_id:$("archives").value}:{});
    feedbackNotice(restore?"存档已恢复，融合暂停。请回到存档时的视角，确认跟踪正常后点击“继续融合”。":"旧地图已自动存档，当前地图已清除，正在从当前视角重新建图。");
  }catch(e){feedbackNotice(e.message,"error")}
  finally{mapBusy=false;poll()}
}
$("clear").addEventListener("click",()=>changeMap(false));
$("restore").addEventListener("click",()=>changeMap(true));
$("archives").addEventListener("change",()=>{if(state)render(state)});
$("end").addEventListener("click",async()=>{
  try{await post("/api/end");ended=true;notice("会话已结束，正在保存地图和清理连接。请观察机器人最终状态。","warning");document.querySelectorAll("button").forEach(b=>b.disabled=true)}
  catch(e){notice(e.message,"error")}
});
window.addEventListener("blur",()=>{pressed.clear();requireRelease=true;heartbeat()});
document.addEventListener("visibilitychange",heartbeat);
window.addEventListener("keydown",e=>{
  if(["INPUT","TEXTAREA","SELECT"].includes(e.target.tagName)||e.target.isContentEditable)return;
  // Native Enter activation on a focused button also repeats while held.
  if(e.key==="Enter"&&(e.repeat||requireRelease)){e.preventDefault();return}
  const k=e.key.toLowerCase();
  if(["w","a","s","d","q","e"].includes(k)){
    e.preventDefault();
    if(e.repeat||pressed.has(k))return;
    const had=pressed.size;pressed.add(k);
    if(requireRelease||had)return;
    action("step",k);
  }else if(e.code==="Space"||e.key==="Escape"){e.preventDefault();if(!e.repeat)action("stop")}
  else if(e.key==="F8"){e.preventDefault();if(!e.repeat)action("arm")}
});
window.addEventListener("keyup",e=>{
  pressed.delete(e.key.toLowerCase());
  if(!pressed.size)requireRelease=false;
});

class CloudViewer{
  constructor(canvas){
    this.canvas=canvas;this.gl=canvas.getContext("webgl",{antialias:true,alpha:false});
    this.count=0;this.points=null;this.trajectory=[];this.yaw=.28;this.pitch=-.12;this.distance=3;this.target=[0,0,-1];this.drag=null;this.auto=true;this.surface=true;this.meshCount=0;
    if(!this.gl){$("map-empty").innerHTML="<strong>WebGL 不可用</strong><span>请在支持 WebGL 的浏览器打开，或保存地图后查看。</span>";return}
    const gl=this.gl;
    const vs="attribute vec3 p;attribute vec3 c;uniform mat4 m;uniform float pointSize;varying vec3 color;void main(){gl_Position=m*vec4(p.x,-p.y,-p.z,1.0);gl_PointSize=pointSize;color=c;}";
    const fs="precision mediump float;varying vec3 color;void main(){gl_FragColor=vec4(color,1.0);}";
    const shader=(type,src)=>{const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(s));return s};
    this.program=gl.createProgram();gl.attachShader(this.program,shader(gl.VERTEX_SHADER,vs));gl.attachShader(this.program,shader(gl.FRAGMENT_SHADER,fs));gl.linkProgram(this.program);
    if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw new Error("WebGL shader link failed");
    this.buffer=gl.createBuffer();this.lines=gl.createBuffer();
    this.meshVertices=gl.createBuffer();this.meshIndices=gl.createBuffer();this.uintIndices=!!gl.getExtension("OES_element_index_uint");
    this.pos=gl.getAttribLocation(this.program,"p");this.col=gl.getAttribLocation(this.program,"c");this.matrix=gl.getUniformLocation(this.program,"m");this.size=gl.getUniformLocation(this.program,"pointSize");
    gl.enable(gl.DEPTH_TEST);gl.clearColor(.067,.098,.137,1);
    canvas.addEventListener("pointerdown",e=>{canvas.setPointerCapture(e.pointerId);this.drag=[e.clientX,e.clientY];this.auto=false});
    canvas.addEventListener("pointermove",e=>{if(!this.drag)return;this.yaw+=(e.clientX-this.drag[0])*.007;this.pitch=Math.max(-1.4,Math.min(1.4,this.pitch+(e.clientY-this.drag[1])*.007));this.drag=[e.clientX,e.clientY]});
    canvas.addEventListener("pointerup",()=>this.drag=null);
    canvas.addEventListener("pointercancel",()=>this.drag=null);
    canvas.addEventListener("wheel",e=>{e.preventDefault();this.distance=Math.max(.08,Math.min(100,this.distance*Math.exp(e.deltaY*.001)));this.auto=false},{passive:false});
    new ResizeObserver(()=>this.resize()).observe(canvas);
    this.resize();this.draw();
  }
  resize(){if(!this.gl)return;const r=this.canvas.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);this.canvas.width=Math.round(r.width*d);this.canvas.height=Math.round(r.height*d);this.gl.viewport(0,0,this.canvas.width,this.canvas.height);if(this.auto)this.fit()}
  clear(){this.count=0;this.points=null;this.trajectory=[];this.auto=true;this.meshCount=0}
  setMesh(vertices,indices){
    if(!this.gl)return;const g=this.gl;
    g.bindBuffer(g.ARRAY_BUFFER,this.meshVertices);
    if(this.uintIndices){
      g.bufferData(g.ARRAY_BUFFER,vertices,g.DYNAMIC_DRAW);g.bindBuffer(g.ELEMENT_ARRAY_BUFFER,this.meshIndices);g.bufferData(g.ELEMENT_ARRAY_BUFFER,indices,g.DYNAMIC_DRAW);
    }else{
      const expanded=new Float32Array(indices.length*6);
      for(let i=0;i<indices.length;i++)expanded.set(vertices.subarray(indices[i]*6,indices[i]*6+6),i*6);
      g.bufferData(g.ARRAY_BUFFER,expanded,g.DYNAMIC_DRAW);
    }
    this.meshCount=indices.length;
  }
  setPoints(data){
    if(!this.gl)return;this.points=data;this.count=data.length/6;
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER,this.buffer);this.gl.bufferData(this.gl.ARRAY_BUFFER,data,this.gl.DYNAMIC_DRAW);
    if(this.auto)this.fit();
  }
  setTrajectory(trajectory){this.trajectory=trajectory}
  fit(){
    if(!this.points||!this.count)return;
    const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];
    for(let i=0;i<this.points.length;i+=6){
      const p=[this.points[i],-this.points[i+1],-this.points[i+2]];
      for(let j=0;j<3;j++){lo[j]=Math.min(lo[j],p[j]);hi[j]=Math.max(hi[j],p[j])}
    }
    this.target=lo.map((v,i)=>(v+hi[i])/2);
    const half=hi.map((v,i)=>(v-lo[i])/2),aspect=this.canvas.width/Math.max(1,this.canvas.height),tan=Math.tan(Math.PI/8);
    const cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch);
    const right=[cy,0,-sy],up=[-sy*sp,cp,-cy*sp],toward=[sy*cp,sp,cy*cp];
    const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
    let needed=.1;
    for(const x of [-1,1])for(const y of [-1,1])for(const z of [-1,1]){
      const corner=[x*half[0],y*half[1],z*half[2]],near=dot(corner,toward);
      needed=Math.max(needed,near+Math.abs(dot(corner,right))/(tan*aspect),near+Math.abs(dot(corner,up))/tan);
    }
    this.distance=Math.max(.3,needed*1.06+.03);
    this.auto=true;
  }
  bind(buffer){
    const g=this.gl;g.bindBuffer(g.ARRAY_BUFFER,buffer);
    g.enableVertexAttribArray(this.pos);g.vertexAttribPointer(this.pos,3,g.FLOAT,false,24,0);
    g.enableVertexAttribArray(this.col);g.vertexAttribPointer(this.col,3,g.FLOAT,false,24,12);
  }
  draw(){
    const g=this.gl;if(!g)return;
    const cp=Math.cos(this.pitch),eye=[this.target[0]+this.distance*Math.sin(this.yaw)*cp,this.target[1]+this.distance*Math.sin(this.pitch),this.target[2]+this.distance*Math.cos(this.yaw)*cp];
    const m=multiply(perspective(Math.PI/4,this.canvas.width/this.canvas.height,.01,200),lookAt(eye,this.target,[0,1,0]));
    g.clear(g.COLOR_BUFFER_BIT|g.DEPTH_BUFFER_BIT);g.useProgram(this.program);g.uniformMatrix4fv(this.matrix,false,new Float32Array(m));g.uniform1f(this.size,Math.min(devicePixelRatio||1,2)*1.8);
    if(this.surface&&this.meshCount){this.bind(this.meshVertices);if(this.uintIndices){g.bindBuffer(g.ELEMENT_ARRAY_BUFFER,this.meshIndices);g.drawElements(g.TRIANGLES,this.meshCount,g.UNSIGNED_INT,0)}else g.drawArrays(g.TRIANGLES,0,this.meshCount)}
    else if(this.count){this.bind(this.buffer);g.drawArrays(g.POINTS,0,this.count)}
    const coords=[];
    const line=(a,b,c)=>{coords.push(...a,...c,...b,...c)};
    line([0,0,0],[.15,0,0],[.95,.3,.3]);line([0,0,0],[0,.15,0],[.25,.8,.4]);line([0,0,0],[0,0,.15],[.3,.5,1]);
    for(let i=1;i<this.trajectory.length;i++)line(this.trajectory[i-1],this.trajectory[i],[.2,.85,.78]);
    g.bindBuffer(g.ARRAY_BUFFER,this.lines);g.bufferData(g.ARRAY_BUFFER,new Float32Array(coords),g.DYNAMIC_DRAW);this.bind(this.lines);g.drawArrays(g.LINES,0,coords.length/6);
    requestAnimationFrame(()=>this.draw());
  }
}
function multiply(a,b){const o=Array(16).fill(0);for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)o[c*4+r]+=a[k*4+r]*b[c*4+k];return o}
function perspective(fovy,aspect,n,f){const t=1/Math.tan(fovy/2);return[t/aspect,0,0,0,0,t,0,0,0,0,(f+n)/(n-f),-1,0,0,2*f*n/(n-f),0]}
function lookAt(eye,target,up){
  const norm=a=>{const d=Math.hypot(...a);return a.map(v=>v/d)},cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  const z=norm(eye.map((v,i)=>v-target[i])),x=norm(cross(up,z)),y=cross(z,x);
  return[x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1];
}
const viewer=new CloudViewer($("cloud"));
heartbeat();poll();setInterval(heartbeat,180);setInterval(poll,250);


