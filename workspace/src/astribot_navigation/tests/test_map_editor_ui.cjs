const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const nodes={},calls=[],images=[];
const state={width:30,height:20,revision:0,radius:.2,undo:0,footprint:[],padding:0,resolution:.05};
const context=new Proxy({},{get:()=>()=>{}});
function element(id){return nodes[id]??=( {value:'0',checked:false,clientWidth:600,clientHeight:400,classList:{toggle(){}},getContext:()=>context,getBoundingClientRect:()=>({left:0,top:0}),setPointerCapture(){},addEventListener(name,fn){this[name]=fn;}});}
const sandbox={document:{getElementById:element},window:{},devicePixelRatio:1,
Image:class{set src(v){images.push(v);queueMicrotask(()=>this.onload())}},
fetch:async(url,opts)=>{if(opts){calls.push(JSON.parse(opts.body));state.revision++;state.undo++}return {ok:true,json:async()=>({...state})}},
console};
vm.createContext(sandbox);vm.runInContext(html.match(/<script>([\s\S]*)<\/script>/)[1],sandbox);
(async()=>{await new Promise(setImmediate);
vm.runInContext('rect=[4,3,7,6]',sandbox);nodes.erase.click();
await new Promise(setImmediate);
assert.equal(calls.length,1);assert.deepEqual(calls[0],{op:'erase',rect:[4,3,7,6],revision:0});
assert(images.includes('/map.png?r=1'));assert(images.includes('/inflation.png?r=1'));
assert(nodes.message.textContent.includes('修改成功'));assert.equal(nodes.undo.disabled,false);
console.log('PASS: erase click sends rectangle and reloads map plus inflation');
})().catch(e=>{console.error(e);process.exitCode=1});
