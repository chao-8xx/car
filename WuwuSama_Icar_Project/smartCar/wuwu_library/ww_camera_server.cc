/*********************************************************************************************************************
 * Wuwu 开源库（Wuwu Open Source Library） — 摄像头模块
 * 版权所有 (c) 2025 Blockingsys
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 本文件是 Wuwu 开源库 的一部分。
 *
 * 本文件按照 GNU 通用公共许可证 第3版（GPLv3）或您选择的任何后续版本的条款授权。
 * 您可以在遵守 GPL-3.0 许可条款的前提下，自由地使用、复制、修改和分发本文件及其衍生作品。
 * 在分发本文件或其衍生作品时，必须以相同的许可证（GPL-3.0）对源代码进行授权并随附许可证副本。
 *
 * 本软件按“原样”提供，不对适销性、特定用途适用性或不侵权做任何明示或暗示的保证。
 * 有关更多细节，请参阅 GNU 官方许可证文本： https://www.gnu.org/licenses/gpl-3.0.html
 *
 * 注：本注释为 GPL-3.0 许可证的中文说明与摘要，不构成法律意见。正式许可以 GPL 原文为准。
 * LICENSE 副本通常位于项目根目录的 LICENSE 文件或 libraries 文件夹下；若未找到，请访问上方链接获取。
 *
 * 额外说明：
 * - 本项目可能包含第三方组件，各组件的版权与许可以其各自随附的 LICENSE 为准；
 * - 分发、修改本文件时请保留本版权与许可声明以尊重原作者权利；
 * - 本文档为中文译述/摘要，英文许可证文本为法律权威版。
 *
 * 文件名称：ww_camera_server.cc
 * 所属模块：wuwu_library
 * 功能描述：摄像头服务器，用于 MJPEG 流与 HTTP 接口
 * 版本信息：详见 libraries/doc/version
 * 开发环境：Linux，GCC / Clang，OpenCV，V4L2
 * 联系/主页：请参阅项目 README
 *
 * 修改记录：
 * 日期        作者              说明
 * 2025-12-16  Blockingsys    添加 GPL-3.0 中文许可头
 ********************************************************************************************************************/

#include "ww_camera_server.h"
#include "music.h"

// ----------------------------------------------------
// 引用全局对象
// ----------------------------------------------------
extern TrackBase track_base;
extern PointState point_state;

// 静态成员初始化
CameraStreamServer* CameraStreamServer::instance = nullptr;

// HTML查看器内容
const char* viewer_html = R"HTML(
<!DOCTYPE html> 
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>摄像头实时查看 (WuwuSama Icar)</title>
    <style>
        body { margin: 0; padding: 20px; background: #1a1a1a; font-family: Arial, sans-serif; }
        .container { max-width: 1200px; margin: 0 auto; background: #2d2d2d; border-radius: 10px; padding: 20px; box-shadow: 0 5px 20px rgba(0,0,0,0.5); }
        h1 { color: #fff; text-align: center; margin-bottom: 20px; }
        
        .layout-row { display: flex; flex-direction: row; gap: 20px; margin-bottom: 20px; }
        
        /* 核心流媒体画板与悬浮容器 */
        #video-wrapper { 
            flex: 2; position: relative; background: #000; border-radius: 8px; border: 1px solid #444; 
            display: flex; justify-content: center; align-items: center; min-height: 600px; overflow: hidden;
        }
        #stream { 
            width: 100%; height: 100%; object-fit: contain; image-rendering: pixelated; z-index: 1; 
        }
        /* 这个画布作为鼠标事件捕获层，完全覆盖在上面并随容器大小缩放 */
        #overlay-canvas { 
            position: absolute; top: 0; left: 0; width: 100%; height: 100%; 
            z-index: 5; cursor: crosshair; object-fit: contain; pointer-events: auto;
        }
        /* 精准悬浮坐标提示框 */
        #coord-tooltip { 
            position: absolute; display: none; background: rgba(0, 255, 128, 0.9); color: #000; 
            font-family: monospace; font-size: 15px; font-weight: bold; padding: 4px 8px; 
            border-radius: 4px; pointer-events: none; z-index: 20; box-shadow: 0 0 10px rgba(0,255,0,0.4);
        }

        #status-panel { 
            flex: 1; height: 80vh; min-height: 600px; background: #000; border-radius: 8px; border: 1px solid #444; 
            padding: 25px; font-family: monospace; font-size: 18px; line-height: 1.6; color: #fff; overflow-y: auto; text-align: left; box-sizing: border-box;
        }
        
        .controls { margin-top: 20px; text-align: center; }
        button { background: #4CAF50; color: white; border: none; padding: 12px 24px; margin: 5px; border-radius: 5px; cursor: pointer; font-size: 16px; transition: background 0.3s; }
        button:hover { background: #45a049; }
        .snapshot-btn { background: #2196F3; }
        .snapshot-btn:hover { background: #0b7dda; }
        
        .info { color: #aaa; margin-top: 15px; font-size: 14px; line-height: 1.6; }
        .hint { color: #f5a623; font-size: 13px; margin-top: 8px; }
        .status { display: inline-block; width: 10px; height: 10px; border-radius: 50%; background: #4CAF50; margin-right: 8px; animation: pulse 2s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        @keyframes blink { 0%, 50% { opacity: 1; } 51%, 100% { opacity: 0.3; } }
        
        .filename-config { margin-top: 15px; text-align: center; }
        .filename-config label { color: #aaa; font-size: 14px; margin-right: 10px; }
        .filename-config input { background: #1a1a1a; color: #fff; border: 1px solid #555; padding: 8px 12px; border-radius: 5px; font-size: 14px; width: 200px; }
        .filename-config input:focus { outline: none; border-color: #4CAF50; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 摄像头实时查看 <span class="status"></span></h1>
        
        <div class="layout-row">
            <!-- 视频流抓取与十字准星悬浮区 -->
            <div id="video-wrapper">
                <div style="position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.7); color: #0f0; padding: 4px 8px; border: 1px solid #0f0; border-radius: 4px; font-size: 12px; z-index: 10;">RAW_VISION 1:1</div>
                <img id="stream" src="/stream" alt="摄像头视频流">
                
                <!-- 占位拦截层，捕获精确的相对坐标 -->
                <canvas id="overlay-canvas" width="320" height="240"></canvas>
                <!-- 跟随鼠标的探针卡尺 -->
                <div id="coord-tooltip">X: 0, Y: 0</div>
            </div>

            <!-- 数据调试侧边栏 -->
            <div id="status-panel">
                <div style="color: #0ff; font-size: 22px; margin-bottom: 20px; font-weight: bold; border-bottom: 2px dashed #ececec; padding-bottom: 10px; text-align: center; letter-spacing: 2px;">=== UNIFIED DEBUG ===</div>
                
                <div style="margin-bottom: 15px;">
                    <div style="color: #fff; font-weight:bold;">--- MODEL_STATE ---</div>
                    <div style="color: #fff;">STATUS     : <span id="ui-model-status">FINDING</span></div>
                    <div style="color: #fff;">RESULT     : <span id="ui-model-result">无</span></div>
                </div>
                
                <div style="margin-bottom: 15px;">
                    <div style="color: #fff; font-weight:bold;">--- PTS_STATE ---</div>
                    <div style="color: #fff;">MID_MODE   : <span id="ui-mid-mode" style="font-weight:bold;">--</span></div>
                    <div style="color: #fff;">PTS_SIZE   : L:<span id="ui-pts-l">0</span> | R:<span id="ui-pts-r">0</span></div>
                    <div style="color: #fff;">MNT_LEN    : L:<span id="ui-mnt-l">0</span> | R:<span id="ui-mnt-r">0</span></div>
                    <div style="color: #fff;">DX_VAR     : L:<span id="ui-var-l">--</span> | R:<span id="ui-var-r">--</span></div>
                    <div style="height: 10px;"></div>
                    <div style="color: #fff;">BREAK_Y    : L: <span id="ui-break-l">--</span> | R: <span id="ui-break-r">--</span></div>
                    <div style="color: #fff;">UP_CRN     : L: <span id="ui-crn-up-l">--</span> | R: <span id="ui-crn-up-r">--</span></div>
                    <div style="color: #fff;">MID_CRN    : L: <span id="ui-crn-mid-l">--</span> | R: <span id="ui-crn-mid-r">--</span></div>
                    <div style="color: #fff;">DN_CRN     : L: <span id="ui-crn-dn-l">--</span> | R: <span id="ui-crn-dn-r">--</span></div>
                </div>
                
                <div style="margin-bottom: 15px;">
                    <div style="color: #fff; font-weight:bold;">--- ELEMENT_STATE ---</div>
                    <div style="color: #fff;">MAIN_STATE : <span id="ui-main-state" style="font-weight:bold; color: #00FFFF;">常规道路</span></div>
                    <div style="color: #fff;">NORMAL_TYPE: <span id="ui-normal-type">--</span></div>
                    <div style="color: #fff;">CROSSROAD  : <span id="ui-crossroad-state">0-NORMAL</span></div>
                    <div style="color: #fff;">CIRCLE     : <span id="ui-circle-state">0</span></div>
                    <div style="color: #fff;">BARRIER    : <span id="ui-barrier-state">0</span></div>
                    <div style="color: #fff;">RAMP       : <span id="ui-ramp-state">0</span></div>
                    <div style="color: #fff;">OUT_BOUNDS : <span id="ui-out-bounds" style="font-weight:bold;">NO</span></div>
                </div>

                <div style="margin-bottom: 15px;">
                    <div style="color: #fff; font-weight:bold;">--- SENSOR_FUSION ---</div>
                    <div style="color: #fff;">TOF_DIST   : <span id="ui-tof-dist">0</span></div>
                    <div style="color: #fff;">DIST     : <span id="ui-motor-distance">0.0</span>cm</div>
                    <div style="color: #fff;">PITCH      : <span id="ui-gyro-pitch">0.0</span>°</div>
                    <div style="color: #fff;">YAW        : <span id="ui-gyro-yaw">0.0</span>°</div>
                    <div style="color: #fff;">YAW_RATE   : <span id="ui-gyro-yaw-rate">0.0</span>°/s</div>
                    <div style="color: #fff;">ENC_SPD    : L:<span id="ui-l-speed">0.0</span> | R:<span id="ui-r-speed">0.0</span></div>
                </div>

                <div style="margin-bottom: 15px;">
                    <div style="color: #fff; font-weight:bold;">--- CONTROL_TRACK ---</div>
                    <div style="color: #fff;">LOOKAHEAD  : Y: <span id="ui-lk-y" style="color: #00FF00;">--</span></div>
                    <div style="color: #fff;">ERROR      : <span id="ui-error" style="font-weight:bold;">0.00</span></div>
                    <div style="color: #fff;">CURVE      : <span id="ui-curve-score">0.00</span></div>
                    <div style="color: #fff;">TARGET_SPD : <span id="ui-target-spd">0.0</span></div>
                </div>
            </div>
        </div>

        <div class="controls">
            <button class="snapshot-btn" onclick="takeSnapshot()">📸 拍照保存</button>
            <button onclick="reconnect()">🔄 重新连接</button>
            <button onclick="toggleFullscreen()">⛶ 全屏视角</button>
        </div>
        <div class="controls" style="margin-top: 10px;">
            <button onclick="playMusic()" style="background: #E91E63;">🎵 播放音乐</button>
            <button onclick="stopMusic()" style="background: #607D8B;">⏹ 停止音乐</button>
        </div>
        <div class="controls" style="margin-top: 10px;">
            <button onclick="playMusic()" style="background: #E91E63;">🎵 播放音乐</button>
            <button onclick="stopMusic()" style="background: #607D8B;">⏹ 停止</button>
        </div>
        <div class="filename-config">
            <label>拍照文件名前缀:</label>
            <input type="text" id="filenamePrefix" value="snapshot" placeholder="snapshot" />
        </div>
        <div class="info">
            <p>• 点击"拍照保存"下载摄像头当前原图 • 支持全屏零延迟检阅 • 视频流: <span id="url"></span></p>
            <p>• 快捷键辅助: [ <strong>K</strong> ] 快速抓拍 | [ <strong>F</strong> ] 切换全屏</p>
            <p>• 网络延时: <strong><span id="latency">--</span></strong> • 实时帧率: <strong><span id="fps">--</span></strong></p>
        </div>
    </div>

    <script>
        document.getElementById('url').textContent = window.location.origin + '/stream';
        const img = document.getElementById('stream');
        
        // 文件名前缀状态托管
        const savedPrefix = localStorage.getItem('filenamePrefix') || 'snapshot';
        document.getElementById('filenamePrefix').value = savedPrefix;
        document.getElementById('filenamePrefix').addEventListener('change', function() {
            localStorage.setItem('filenamePrefix', this.value.trim() || 'snapshot');
        });
        
        // ========== 1:1 Canvas 像素探针模块 ==========
        const canvas = document.getElementById('overlay-canvas');
        const tooltip = document.getElementById('coord-tooltip');
        
        // 软视场/硬视场的恒定物理范围
        const IMG_W = 320;
        const IMG_H = 240;

        canvas.addEventListener('mousemove', function(e) {
            const rect = canvas.getBoundingClientRect();
            // 在 object-fit: contain 约束下，计算实际画面的缩放比以及留白偏移
            const scale = Math.min(rect.width / IMG_W, rect.height / IMG_H);
            const actualW = IMG_W * scale;
            const actualH = IMG_H * scale;
            
            const offsetX = (rect.width - actualW) / 2;
            const offsetY = (rect.height - actualH) / 2;
            
            // 剔除空白区
            const px = e.clientX - rect.left - offsetX;
            const py = e.clientY - rect.top - offsetY;
            
            if (px >= 0 && px < actualW && py >= 0 && py < actualH) {
                const rx = Math.floor(px / scale);
                const ry = Math.floor(py / scale);
                
                tooltip.style.display = 'block';
                tooltip.style.left = (e.clientX - rect.left + 15) + 'px';
                tooltip.style.top = (e.clientY - rect.top + 15) + 'px';
                tooltip.innerHTML = `[${rx}, ${ry}]`;
            } else {
                tooltip.style.display = 'none';
            }
        });
        
        canvas.addEventListener('mouseleave', () => { tooltip.style.display = 'none'; });

        // ========== 功能按钮绑定 ==========
        function takeSnapshot() {
            const prefix = document.getElementById('filenamePrefix').value.trim() || 'snapshot';
            const now = new Date();
            const ts = `${now.getFullYear()}${String(now.getMonth()+1).padStart(2,'0')}${String(now.getDate()).padStart(2,'0')}_${String(now.getHours()).padStart(2,'0')}${String(now.getMinutes()).padStart(2,'0')}${String(now.getSeconds()).padStart(2,'0')}`;
            const filename = `${prefix}_${ts}.jpg`;
            
            const a = document.createElement('a');
            a.href = `/snapshot?prefix=${encodeURIComponent(prefix)}`;
            a.download = filename;
            document.body.appendChild(a); a.click(); document.body.removeChild(a);
        }

        function playMusic() {
            fetch('/music/play/qingtian')
                .then(resp => {
                    if(resp.ok) console.log("Music started");
                    else console.error("Failed to start music");
                })
                .catch(err => console.error("Error:", err));
        }

        function stopMusic() {
            fetch('/music/stop')
                .then(resp => console.log("Music stopped"))
                .catch(err => console.error("Error:", err));
        }
        
        function reconnect() { img.src = '/stream?t=' + new Date().getTime(); }
        function playMusic() { fetch('/music/play/qingtian').catch(()=>{}); }
        function stopMusic() { fetch('/music/stop').catch(()=>{}); }
        function toggleFullscreen() {
            const wrapper = document.getElementById('video-wrapper');
            if (!document.fullscreenElement) wrapper.requestFullscreen();
            else document.exitFullscreen();
        }

        document.addEventListener('keydown', function(e) {
            if (document.activeElement?.tagName === 'INPUT') return;
            if (e.key.toLowerCase() === 'k') { e.preventDefault(); takeSnapshot(); }
            if (e.key.toLowerCase() === 'f') { e.preventDefault(); toggleFullscreen(); }
            if (e.key.toLowerCase() === 'r') { e.preventDefault(); reconnect(); }
        });

        // ========== 状态大屏超低延迟轮询 ==========
        async function fetchCarState() {
            try {
                const res = await fetch("/debug_data");
                if (res.ok) {
                    const data = await res.json();
                    
                    const trackTypes = ['常规','十字路口','圆环','斑马线','坡道','路障','出界'];
                    const trackColors = ['#00FFFF','#FF7F00','#FFD700','#8B00FF','#00FF00','#0000FF','#FF0000'];
                    
                    if(data.track_type !== undefined) {
                        // 【修改点1】：保留 MODEL_STATE (AI检测)
                        let modelStatusEl = document.getElementById('ui-model-status');
                        let modelResultEl = document.getElementById('ui-model-result');
                        if (data.model_detected) {
                            modelStatusEl.textContent = "FOUND"; modelStatusEl.style.color = "#00FF00";
                            modelResultEl.textContent = data.model_class || "未知"; modelResultEl.style.color = "#00FF00";
                        } else {
                            modelStatusEl.textContent = "FINDING"; modelStatusEl.style.color = "#FFFFFF";
                            modelResultEl.textContent = "无"; modelResultEl.style.color = "#FFFFFF";
                        }

                        // 【修改点2】：MID_MODE 动态着色 (绿/蓝/红)
                        let midModeEl = document.getElementById('ui-mid-mode');
                        let mode = data.mid_mode || "UNKNOWN";
                        midModeEl.textContent = mode;
                        if (mode === "BOTH") midModeEl.style.color = "#00FF00";         // 双边：绿色
                        else if (mode === "L_HALF") midModeEl.style.color = "#00BFFF";  // 左单边：蓝色
                        else if (mode === "R_HALF") midModeEl.style.color = "#FF3333";  // 右单边：红色
                        else midModeEl.style.color = "#FFFFFF";

                        // 点数
                        document.getElementById('ui-pts-l').textContent = data.raw_l_pts_size || 0;
                        document.getElementById('ui-pts-r').textContent = data.raw_r_pts_size || 0;
                        document.getElementById('ui-mnt-l').textContent = data.l_mnt_len ?? 0;
                        document.getElementById('ui-mnt-r').textContent = data.r_mnt_len ?? 0;

                        const lVarEl = document.getElementById('ui-var-l');
                        const rVarEl = document.getElementById('ui-var-r');
                        const lVar = Number(data.l_dx_var);
                        const rVar = Number(data.r_dx_var);
                        lVarEl.textContent = Number.isFinite(lVar) ? lVar.toFixed(2) : '--';
                        rVarEl.textContent = Number.isFinite(rVar) ? rVar.toFixed(2) : '--';

                        // 【修改点3】：BREAK_Y 丢线红色显示
                        let breakLEl = document.getElementById('ui-break-l');
                        if (data.l_lost) breakLEl.innerHTML = `<span style="color:#FF0000; font-weight:bold;">LOST(${data.l_lost_y})</span>`; // 红色
                        else breakLEl.innerHTML = `<span style="color:#00FF00;">OK</span>`;

                        let breakREl = document.getElementById('ui-break-r');
                        if (data.r_lost) breakREl.innerHTML = `<span style="color:#FF0000; font-weight:bold;">LOST(${data.r_lost_y})</span>`; // 红色
                        else breakREl.innerHTML = `<span style="color:#00FF00;">OK</span>`;

                        // 【修改点4】：上角点紫色显示
                        let crnUpLEl = document.getElementById('ui-crn-up-l');
                        if (data.l_crn_up) crnUpLEl.innerHTML = `<span style="color:#BA55D3; font-weight:bold;">YES(${data.l_crn_up_y})</span>`; // 紫色
                        else crnUpLEl.innerHTML = `<span style="color:#555;">NO</span>`;

                        let crnUpREl = document.getElementById('ui-crn-up-r');
                        if (data.r_crn_up) crnUpREl.innerHTML = `<span style="color:#BA55D3; font-weight:bold;">YES(${data.r_crn_up_y})</span>`; // 紫色
                        else crnUpREl.innerHTML = `<span style="color:#555;">NO</span>`;
                        
                        // 【修改点5】：中角点橙色显示
                        let crnMidLEl = document.getElementById('ui-crn-mid-l');
                        if (data.l_crn_mid) crnMidLEl.innerHTML = `<span style="color:#FFA500; font-weight:bold;">YES(${data.l_crn_mid_y})</span>`; // 橙色
                        else crnMidLEl.innerHTML = `<span style="color:#555;">NO</span>`;

                        let crnMidREl = document.getElementById('ui-crn-mid-r');
                        if (data.r_crn_mid) crnMidREl.innerHTML = `<span style="color:#FFA500; font-weight:bold;">YES(${data.r_crn_mid_y})</span>`; // 橙色
                        else crnMidREl.innerHTML = `<span style="color:#555;">NO</span>`;
                        
                        // 【修改点6】：下角点黄色显示
                        let crnDnLEl = document.getElementById('ui-crn-dn-l');
                        if (data.l_crn_down) crnDnLEl.innerHTML = `<span style="color:#FFFF00; font-weight:bold;">YES(${data.l_crn_down_y})</span>`; // 黄色
                        else crnDnLEl.innerHTML = `<span style="color:#555;">NO</span>`; 

                        let crnDnREl = document.getElementById('ui-crn-dn-r');
                        if (data.r_crn_down) crnDnREl.innerHTML = `<span style="color:#FFFF00; font-weight:bold;">YES(${data.l_crn_down_y})</span>`; // 黄色
                        else crnDnREl.innerHTML = `<span style="color:#555;">NO</span>`;

                        // 【修改点4】：MAIN_STATE 动态着色
                        let mainStateEl = document.getElementById('ui-main-state');
                        let t_type = data.track_type;
                        if (t_type >= 0 && t_type < trackTypes.length) {
                            mainStateEl.textContent = trackTypes[t_type];
                            mainStateEl.style.color = trackColors[t_type]; // 常规青色，十字橙色...
                        }

                        // 十字路口状态与常规元素
                        document.getElementById('ui-crossroad-state').textContent = data.cross_state || "0-NORMAL";
                        if (data.cross_state && data.cross_state !== "0-NORMAL") {
                            document.getElementById('ui-crossroad-state').style.color = "#FF9900"; // 触发十字时橙色警报
                        } else {
                            document.getElementById('ui-crossroad-state').style.color = "#FFF";
                        }
                        
                        // 常规道路类型显示（强直道/弱直道/弯道）
                        let normalTypeEl = document.getElementById('ui-normal-type');
                        let normalType = data.normal_type || "0-UNKNOWN";
                        normalTypeEl.textContent = normalType;
                        if (normalType === "1-STRAIGHT") {
                            normalTypeEl.style.color = "#00FF00";  // 强直道：绿色
                        } else if (normalType === "2-WEAK_STR") {
                            normalTypeEl.style.color = "#FFFF00";  // 弱直道：黄色
                        } else if (normalType === "3-CURVE") {
                            normalTypeEl.style.color = "#FFA500";  // 弯道：橙色
                        } else {
                            normalTypeEl.style.color = "#888";     // 未知：灰色
                        }
                        
                        document.getElementById('ui-circle-state').textContent = data.circle_state || "0-NONE";
                        document.getElementById('ui-barrier-state').textContent = data.barrier_state || "0-NONE";
                        document.getElementById('ui-ramp-state').textContent = data.ramp_state || "0-NONE";
                        
                        // 出界状态显示（醒目红色闪烁警告）
                        let outBoundsEl = document.getElementById('ui-out-bounds');
                        if (data.is_out) {
                            outBoundsEl.textContent = "⚠ OUT! ⚠";
                            outBoundsEl.style.color = "#FF0000";
                            outBoundsEl.style.fontWeight = "bold";
                            outBoundsEl.style.fontSize = "20px";
                            outBoundsEl.style.animation = "blink 0.5s infinite";
                        } else {
                            outBoundsEl.textContent = "NO";
                            outBoundsEl.style.color = "#00FF00";
                            outBoundsEl.style.fontWeight = "normal";
                            outBoundsEl.style.fontSize = "18px";
                            outBoundsEl.style.animation = "none";
                        }

                        // 传感器
                        document.getElementById('ui-tof-dist').textContent = data.tof_dist !== undefined ? data.tof_dist : '0';
                        const pitchEl = document.getElementById('ui-gyro-pitch');
                        if (pitchEl) pitchEl.textContent = data.gyro_pitch !== undefined ? Number(data.gyro_pitch).toFixed(1) : '0.0';
                        document.getElementById('ui-motor-distance').textContent = data.motor_distance !== undefined ? Number(data.motor_distance).toFixed(1) : '0.0';
                        document.getElementById('ui-gyro-yaw').textContent = data.gyro_yaw !== undefined ? Number(data.gyro_yaw).toFixed(1) : '0.0';
                        document.getElementById('ui-gyro-yaw-rate').textContent = data.gyro_yaw_rate !== undefined ? Number(data.gyro_yaw_rate).toFixed(1) : '0.0';
                        document.getElementById('ui-l-speed').textContent = data.l_speed !== undefined ? Number(data.l_speed).toFixed(1) : '0.0';
                        document.getElementById('ui-r-speed').textContent = data.r_speed !== undefined ? Number(data.r_speed).toFixed(1) : '0.0';

                        // 【修改点5】：ERROR 颜色区分与控制输出
                        document.getElementById('ui-lk-y').textContent = data.lk_y !== undefined ? data.lk_y : '--';
                        
                        let errEl = document.getElementById('ui-error');
                        let errVal = Number(data.err_total || 0);
                        errEl.textContent = errVal.toFixed(2);
                        if (errVal < -0.1) errEl.style.color = "#00BFFF"; // 负数(左偏): 蓝色
                        else if (errVal > 0.1) errEl.style.color = "#FF3333"; // 正数(右偏): 红色
                        else errEl.style.color = "#00FF00"; // 接近0(居中): 绿色

                        document.getElementById('ui-curve-score').textContent = Number(data.curve_score || 0).toFixed(2);
                        document.getElementById('ui-target-spd').textContent = Number(data.target_speed || 0).toFixed(1);
                    }
                }
            } catch (err) {}
            setTimeout(fetchCarState, 100); 
        }
        
        async function updatePerformanceStats() {
            try {
                const response = await fetch('/stats');
                if (response.ok) {
                    const data = await response.json();
                    if (data.estimatedFps) document.getElementById('fps').textContent = Number(data.estimatedFps).toFixed(1) + ' FPS';
                    if (data.latestCaptureTsMs && data.serverTsMs) {
                       document.getElementById('latency').textContent = Math.max(0, Number(data.serverTsMs) - Number(data.latestCaptureTsMs)) + ' ms(板载)';
                    }
                }
            } catch (err) {}
        }

        fetchCarState();
        setInterval(updatePerformanceStats, 1000);
    </script>
</body>
</html>
)HTML";

CameraStreamServer::CameraStreamServer(void)
    : server_sock_fd(-1)
    , server_port(CAMERA_STREAM_DEFAULT_PORT)
    , running(false)
    , server_thread_id(0)
    , latest_frame_id(0)
    , latest_capture_ts_ms(0)
    , ema_fps(0.0)
    , music_player(nullptr)
{
    pthread_mutex_init(&frame_mutex, NULL);
    pthread_cond_init(&frame_cond, NULL);
    pthread_mutex_init(&sock_mutex, NULL);
    // pthread_mutex_init(&original_frame_mutex, NULL);
}

CameraStreamServer::CameraStreamServer(AsyncMusicPlayer* music)
    : CameraStreamServer()
{
    music_player = music;
}

CameraStreamServer::~CameraStreamServer(void)
{
    stop_server();
    pthread_mutex_destroy(&frame_mutex);
    pthread_cond_destroy(&frame_cond);
    pthread_mutex_destroy(&sock_mutex);
    
}


/*******************************************************************
 * @brief       获取本机IP地址
 * 
 * @return      返回本机IP地址字符串
 * 
 * @note        自动选择优先级最高的网络接口IP
 ******************************************************************/
std::string CameraStreamServer::get_local_ip(void)
{
    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];
    
    if (getifaddrs(&ifaddr) == -1) {
        return "127.0.0.1";
    }
    
    std::string result = "127.0.0.1";
    std::string fallback_ip = "";
    int best_priority = -1;
    
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;
        
        int family = ifa->ifa_addr->sa_family;
        
        // 只处理IPv4地址
        if (family == AF_INET) {
            // 跳过回环地址
            if (strcmp(ifa->ifa_name, "lo") == 0) continue;
            
            int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                              host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0) continue;
            
            // 计算接口优先级
            int priority = 0;
            bool is_up = (ifa->ifa_flags & IFF_UP) != 0;
            bool is_running = (ifa->ifa_flags & IFF_RUNNING) != 0;
            
            // UP 且 RUNNING 的接口优先级最高
            if (is_up && is_running) {
                priority = 100;
                // wlan/eth 接口额外加分
                if (strncmp(ifa->ifa_name, "wlan", 4) == 0) priority += 20;
                else if (strncmp(ifa->ifa_name, "eth", 3) == 0) priority += 15;
                else if (strncmp(ifa->ifa_name, "en", 2) == 0) priority += 15; // macOS/BSD
                else priority += 5; // 其他接口
            } 
            // 只有 UP 没有 RUNNING 的接口作为备选
            else if (is_up) {
                priority = 50;
                if (strncmp(ifa->ifa_name, "wlan", 4) == 0) priority += 10;
                else if (strncmp(ifa->ifa_name, "eth", 3) == 0) priority += 8;
                else if (strncmp(ifa->ifa_name, "en", 2) == 0) priority += 8;
            }
            // 其他情况优先级很低
            else {
                priority = 10;
            }
            
            // 选择优先级最高的接口
            if (priority > best_priority) {
                best_priority = priority;
                result = host;
            }
            
            // 保存第一个有效IP作为最终备选
            if (fallback_ip.empty() && strcmp(host, "127.0.0.1") != 0) {
                fallback_ip = host;
            }
        }
    }
    
    freeifaddrs(ifaddr);
    
    // 如果没找到合适的，使用备选IP
    if (result == "127.0.0.1" && !fallback_ip.empty()) {
        result = fallback_ip;
    }
    
    return result;
}

/*******************************************************************
 * @brief       关闭服务器socket
 * 
 * @note        线程安全的关闭操作
 ******************************************************************/
void CameraStreamServer::close_server_socket(void)
{
    pthread_mutex_lock(&sock_mutex);
    if (server_sock_fd >= 0) {
        shutdown(server_sock_fd, SHUT_RDWR);
        close(server_sock_fd);
        server_sock_fd = -1;
    }
    pthread_mutex_unlock(&sock_mutex);
}

/*******************************************************************
 * @brief       获取当前时间戳(毫秒)
 * 
 * @return      返回当前时间戳(毫秒)
 ******************************************************************/
uint64_t CameraStreamServer::now_ms(void)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/*******************************************************************
 * @brief       格式化时间戳
 * 
 * @param       ts_ms           时间戳(毫秒)
 * 
 * @return      返回格式化后的时间字符串
 ******************************************************************/
std::string CameraStreamServer::format_timestamp(uint64_t ts_ms)
{
    if (ts_ms == 0) return "--";
    time_t seconds = static_cast<time_t>(ts_ms / 1000);
    int ms = static_cast<int>(ts_ms % 1000);
    struct tm tm_time;
    localtime_r(&seconds, &tm_time);
    char date_buf[64];
    strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S", &tm_time);
    char buf[80];
    snprintf(buf, sizeof(buf), "%s.%03d", date_buf, ms);
    return std::string(buf);
}

/*******************************************************************
 * @brief       发送HTTP响应
 * 
 * @param       sock            客户端socket
 * @param       content_type    内容类型
 * @param       body            响应体
 * @param       body_len        响应体长度
 ******************************************************************/
void CameraStreamServer::send_response(int sock, const char* content_type, const char* body, size_t body_len)
{
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n";
    header << "Content-Type: " << content_type << "\r\n";
    header << "Cache-Control: no-cache, no-store, must-revalidate\r\n";
    header << "Content-Length: " << body_len << "\r\n";
    header << "Connection: close\r\n\r\n";
    std::string h = header.str();
    send(sock, h.c_str(), h.length(), 0);
    send(sock, body, body_len, 0);
}

/*******************************************************************
 * @brief       发送统计信息响应
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void CameraStreamServer::send_stats_response(int sock)
{
    uint64_t capture_ts = latest_capture_ts_ms;
    uint64_t frame_id = 0;
    pthread_mutex_lock(&frame_mutex);
    frame_id = latest_frame_id;
    pthread_mutex_unlock(&frame_mutex);

    uint64_t server_ts = now_ms();
    double fps = ema_fps;
    std::ostringstream body;
    body << std::fixed << std::setprecision(2)
         << "{\"latestFrameId\":" << frame_id
         << ",\"latestCaptureTsMs\":" << capture_ts
         << ",\"serverTsMs\":" << server_ts
         << ",\"estimatedFps\":" << fps << "}";
    std::string json = body.str();
    send_response(sock, "application/json; charset=utf-8", json.c_str(), json.size());
}

/*******************************************************************
 * @brief       发送MJPEG流
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void CameraStreamServer::send_mjpeg_stream(int sock)
{
    const char* header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    send(sock, header, strlen(header), 0);
    
    uint64_t last_frame_sent = 0;

    std::vector<unsigned char> jpeg_copy;

    while (running) {
        pthread_mutex_lock(&frame_mutex);
        while (running && latest_frame_id == last_frame_sent) {
            pthread_cond_wait(&frame_cond, &frame_mutex);
        }

        if (!running) {
            pthread_mutex_unlock(&frame_mutex);
            break;
        }

        if (current_jpeg.empty()) {
            pthread_mutex_unlock(&frame_mutex);
            continue;
        }

        jpeg_copy = current_jpeg;
        last_frame_sent = latest_frame_id;
        pthread_mutex_unlock(&frame_mutex);
        
        char boundary[256];
        snprintf(boundary, sizeof(boundary),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                jpeg_copy.size());

        static int send_shrink_counter = 0;
        if (++send_shrink_counter >= 120) {
            jpeg_copy.shrink_to_fit();
            send_shrink_counter = 0;
        }
        
        if (send(sock, boundary, strlen(boundary), 0) < 0) break;
        if (send(sock, jpeg_copy.data(), jpeg_copy.size(), 0) < 0) break;
        if (send(sock, "\r\n", 2, 0) < 0) break;
    }
}

/*******************************************************************
 * @brief       处理拍照请求(将当前JPEG帧转为PNG并发送到客户端)
 * 
 * @param       sock            客户端socket
 * @param       prefix          文件名前缀
 ******************************************************************/
void CameraStreamServer::handle_snapshot_request(int sock, const std::string& prefix)
{   
    // 获取当前图像数据：如果使用的是 update_frame_mat，则优先取未编码的 current_frame（零拷贝）；
    // 否则如果使用的是零拷贝图传 update_frame_jpeg，则只能从 current_jpeg 解析。
    pthread_mutex_lock(&frame_mutex);
    cv::Mat frame_copy = current_frame;
    std::vector<unsigned char> jpeg_copy;
    if (frame_copy.empty()) {
        jpeg_copy = current_jpeg;
    }
    pthread_mutex_unlock(&frame_mutex);
    
    if (frame_copy.empty()) {
        if (jpeg_copy.empty()) {
            const char* error_html = "<h1>Error</h1><p>没有可用的图像帧</p>";
            send_response(sock, "text/html; charset=utf-8", error_html, strlen(error_html));
            return;
        }
        // 如果没有 RGB 原始帧（使用了纯图传方案），则从保存的 JPEG 解码为 Mat
        frame_copy = cv::imdecode(jpeg_copy, cv::IMREAD_UNCHANGED);
        if (frame_copy.empty()) {
            const char* error_html = "<h1>Error</h1><p>JPEG解码失败，无法导出PNG</p>";
            send_response(sock, "text/html; charset=utf-8", error_html, strlen(error_html));
            return;
        }
    }

    // 直接从原始frame编码为PNG，避免 JPEG→解码→PNG 的多余编解码步骤
    std::vector<unsigned char> png_buffer;
    std::vector<int> png_params;
    png_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    png_params.push_back(0);  // PNG压缩等级0：最快，体积较大
    if (!cv::imencode(".png", frame_copy, png_buffer, png_params)) {
        const char* error_html = "<h1>Error</h1><p>PNG编码失败</p>";
        send_response(sock, "text/html; charset=utf-8", error_html, strlen(error_html));
        return;
    }

    // 生成文件名（使用自定义前缀）
    time_t now = time(NULL);
    struct tm tm_time;
    localtime_r(&now, &tm_time);
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_%04d%02d%02d_%02d%02d%02d.png",
             prefix.c_str(),
             tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
             tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
    
    // 发送HTTP响应头（发送PNG）
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n";
    header << "Content-Type: image/png\r\n";
    header << "Content-Length: " << png_buffer.size() << "\r\n";
    header << "Content-Disposition: attachment; filename=\"" << filename << "\"\r\n";
    header << "Cache-Control: no-cache\r\n";
    header << "Connection: close\r\n\r\n";
    
    std::string h = header.str();
    send(sock, h.c_str(), h.length(), 0);
    send(sock, png_buffer.data(), png_buffer.size(), 0);
    
    std::cout << "✓ 已发送PNG图片到客户端: " << filename
              << " (大小: " << png_buffer.size() / 1024 << " KB, 源自原始frame)" << std::endl;
}

/*******************************************************************
 * @brief       处理音乐控制请求
 * 
 * @param       sock            客户端socket
 * @param       path            请求路径
 ******************************************************************/
void CameraStreamServer::handle_music_request(int sock, const std::string& path)
{
    if (!music_player) {
         const char* msg = "Music player not initialized";
         send_response(sock, "text/plain", msg, strlen(msg));
         return;
    }

    if (path.find("/music/play") == 0) {
        // 播放《晴天》
        // 如果有更多歌曲，可以在这里解析URL path
        music_player->play(qing_tian, sizeof(qing_tian)/sizeof(Note));
        
        const char* msg = "Playing music";
        send_response(sock, "text/plain", msg, strlen(msg));
        std::cout << "收到音乐播放请求: 晴天" << std::endl;
    } 
    else if (path.find("/music/stop") == 0) {
        music_player->stop();
        
        const char* msg = "Stopped music";
        send_response(sock, "text/plain", msg, strlen(msg));
        std::cout << "收到音乐停止请求" << std::endl;
    }
    else {
        const char* msg = "Unknown music command";
        send_response(sock, "text/plain", msg, strlen(msg));
    }
}

/*******************************************************************
 * @brief       处理客户端HTTP请求
 * 
 * @param       sock            客户端socket
 ******************************************************************/
void CameraStreamServer::handle_client_request(int sock)
{
    char buffer[4096];
    ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) {
        close(sock);
        return;
    }
    buffer[n] = '\0';
    
    // 解析请求路径
    std::string request(buffer);
    size_t path_start = request.find(" ") + 1;
    size_t path_end = request.find(" ", path_start);
    std::string path = request.substr(path_start, path_end - path_start);
    
    if (path == "/" || path.find("/viewer") == 0 || path.find("/?") == 0) {
        // 返回HTML查看器
        send_response(sock, "text/html; charset=utf-8", viewer_html, strlen(viewer_html));
    } else if (path.find("/stream") == 0) {
        // 返回视频流
        send_mjpeg_stream(sock);
    } else if (path.find("/stats") == 0) {
        send_stats_response(sock);
    } else if (path.find("/snapshot") == 0) {
        // 解析文件名前缀参数
        std::string prefix = "snapshot";  // 默认前缀
        size_t query_pos = path.find("?prefix=");
        if (query_pos != std::string::npos) {
            size_t prefix_start = query_pos + 8;  // "?prefix=" 长度为8
            size_t prefix_end = path.find("&", prefix_start);
            if (prefix_end == std::string::npos) {
                prefix_end = path.length();
            }
            prefix = path.substr(prefix_start, prefix_end - prefix_start);
            
            // URL解码（简单处理，只处理常见字符）
            size_t pos = 0;
            while ((pos = prefix.find("%20", pos)) != std::string::npos) {
                prefix.replace(pos, 3, " ");
                pos += 1;
            }
            
            // 安全检查：只允许字母、数字、下划线、中划线
            bool valid = true;
            for (char c : prefix) {
                if (!isalnum(c) && c != '_' && c != '-') {
                    valid = false;
                    break;
                }
            }
            if (!valid || prefix.empty()) {
                prefix = "snapshot";
            }
        }
        
        // 处理拍照请求
        handle_snapshot_request(sock, prefix);
    } else if (path.find("/music") == 0) {
        handle_music_request(sock, path);
    } else if (path.find("/debug_data") == 0) {
        // 返回调试数据 (JSON格式)
        std::string json_str = preprocess.pack_debug_data();
        
        std::string response = "HTTP/1.1 200 OK\r\n"
                               "Content-Type: application/json\r\n"
                               "Connection: close\r\n"
                               "Access-Control-Allow-Origin: *\r\n\r\n" + json_str;
                               
        send(sock, response.c_str(), response.length(), 0);
    } else {
        // 404
        const char* not_found = "<h1>404 Not Found</h1>";
        send_response(sock, "text/html", not_found, strlen(not_found));
    }
    
    close(sock);
}

/*******************************************************************
 * @brief       客户端处理线程上下文结构体
 * 
 * @note        用于在线程间传递服务器实例和客户端socket
 ******************************************************************/
struct ClientThreadContext {
    CameraStreamServer* server_instance;
    int client_socket_fd;
};

/*******************************************************************
 * @brief       客户端处理线程函数
 * 
 * @param       arg             ClientThreadContext指针
 * 
 * @return      返回NULL
 * 
 * @note        每个客户端连接创建独立线程处理
 ******************************************************************/
void* CameraStreamServer::client_thread_func(void* arg)
{
    ClientThreadContext* context = static_cast<ClientThreadContext*>(arg);
    if (!context) return NULL;
    
    CameraStreamServer* server = context->server_instance;
    int sock = context->client_socket_fd;
    delete context;
    
    if (server) {
        server->handle_client_request(sock);
    } else {
        close(sock);
    }
    
    return NULL;
}

/*******************************************************************
 * @brief       服务器线程函数
 * 
 * @param       arg             CameraStreamServer实例指针
 * 
 * @return      返回NULL
 ******************************************************************/
void* CameraStreamServer::server_thread_func(void* arg)
{
    CameraStreamServer* server = static_cast<CameraStreamServer*>(arg);
    if (!server) return NULL;
    
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        std::cerr << "创建socket失败" << std::endl;
        return NULL;
    }

    pthread_mutex_lock(&server->sock_mutex);
    server->server_sock_fd = server_sock;
    pthread_mutex_unlock(&server->sock_mutex);
    
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(server->server_port);
    
    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "绑定端口失败" << std::endl;
        close(server_sock);
        return NULL;
    }
    
    if (listen(server_sock, 10) < 0) {
        std::cerr << "监听失败" << std::endl;
        close(server_sock);
        return NULL;
    }
    
    // 获取本机IP地址
    std::string local_ip = server->get_local_ip();
    
    std::cout << "\n======================================" << std::endl;
    std::cout << "📡 MJPEG摄像头图传服务器启动成功!" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "监听端口: " << server->server_port << std::endl;
    std::cout << "本机IP: " << local_ip << std::endl;
    std::cout << "请在浏览器访问: http://" << local_ip << ":" << server->server_port << std::endl;
    std::cout << "======================================\n" << std::endl;
    
    while (server->running) {
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &len);
        
        if (client_sock < 0) {
            if (!server->running) break;
            continue;
        }
        
        // 优化socket选项
        int flag = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // 设置发送缓冲区大小
        int sndbuf = 65536;
        setsockopt(client_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        // 创建客户端线程处理上下文，传递当前服务器实例
        pthread_t client_thread_id;
        ClientThreadContext* client_context = new ClientThreadContext{
            .server_instance = server,
            .client_socket_fd = client_sock
        };
        pthread_create(&client_thread_id, NULL, client_thread_func, client_context);
        pthread_detach(client_thread_id);
    }
    
    server->close_server_socket();
    return NULL;
}

/*******************************************************************
 * @brief       信号处理函数
 * 
 * @param       sig             信号值
 ******************************************************************/
void CameraStreamServer::signal_handler(int sig)
{
    std::cout << "\n正在关闭服务器..." << std::endl;
    if (instance) {
        instance->stop_server();
    }
}

/*******************************************************************
 * @brief       启动摄像头图传服务器
 * 
 * @param       port            服务器监听端口(默认8080)
 * 
 * @return      返回启动状态
 * @retval      0               启动成功
 * @retval      -1              启动失败
 * 
 * @example     //启动摄像头图传服务器
 *              if(camera_server.start_server(8080) < 0) {
 *                  return -1;
 *              }
 * 
 * @note        在后台线程中启动HTTP服务器，支持浏览器访问
 *              访问 http://<开发板IP>:<port> 即可查看实时画面
 ******************************************************************/
int CameraStreamServer::start_server(int port)
{
    if (running) {
        std::cout << "服务器已经在运行中" << std::endl;
        return 0;
    }
    
    server_port = port;
    running = true;
    latest_frame_id = 0;
    current_jpeg.clear();
    
    // 设置全局实例指针(用于信号处理)
    instance = this;
    
    // [fix] 库内部不再强行接管 SIGINT/SIGTERM。
    // 应用层（run/main.cc）已经有更可靠的退出控制（包括停止采集/唤醒线程等）。
    // 如果这里再注册 signal，会导致 Ctrl+C 同时触发多套退出逻辑，表现为重复打印“正在关闭服务器...”，
    // 且在某些终端下更容易出现“看起来退不出去”的错觉。
    // 如需信号退出，请在应用层调用 stop_server()。

    if (pthread_create(&server_thread_id, NULL, server_thread_func, this) != 0) {
        std::cerr << "创建服务器线程失败" << std::endl;
        running = false;
        return -1;
    }
    pthread_detach(server_thread_id);
    
    std::cout << "摄像头图传服务器启动中..." << std::endl;
    return 0;
}

/*******************************************************************
 * @brief       更新摄像头帧数据
 * 
 * @param       frame           OpenCV Mat格式的图像帧
 * 
 * @example     camera_server.update_frame(frame);
 * 
 * @note        将最新的摄像头帧推送到服务器，供客户端获取
 *              自动编码为JPEG格式并计算帧率
 ******************************************************************/
void CameraStreamServer::update_frame_mat(const cv::Mat& frame)
{
    // 确保Mat数据有效
    if (frame.empty()) {
        std::cerr << "错误: update_frame_mat() 接收到空Mat图像" << std::endl;
        return;
    }

    uint64_t capture_ts_ms = now_ms();
    
    // 计算FPS
    static uint64_t last_capture_ts_local = 0;
    static double local_fps_estimate = 0.0;
    if (last_capture_ts_local != 0) {
        uint64_t delta = capture_ts_ms - last_capture_ts_local;
        if (delta > 0) {
            double instant_fps = 1000.0 / static_cast<double>(delta);
            if (local_fps_estimate <= 0.0) {
                local_fps_estimate = instant_fps;
            } else {
                local_fps_estimate = 0.85 * local_fps_estimate + 0.15 * instant_fps;
            }
            ema_fps = local_fps_estimate;
        }
    }
    last_capture_ts_local = capture_ts_ms;

    // 编码为JPEG（低质量，用于图传）
    std::vector<unsigned char> jpeg_buffer;
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(95); // 质量90
    
    if (cv::imencode(".jpg", frame, jpeg_buffer, params)) {
        latest_capture_ts_ms = capture_ts_ms;
        pthread_mutex_lock(&frame_mutex);
        current_jpeg.swap(jpeg_buffer);
        current_frame = frame;  // 保持原始帧数据，供拍照使用
        ++latest_frame_id;
        pthread_cond_broadcast(&frame_cond);
        pthread_mutex_unlock(&frame_mutex);
    }
}

/*******************************************************************
 * @brief       更新摄像头帧数据（零拷贝JPEG）
 * 
 * @param       jpeg_data       JPEG原始数据
 * 
 * @example     camera.capture_frame(frame, false);  // 不解码
 *              CameraStreamServer.update_frame_jpeg(camera.jpeg_nowdata);
 * 
 * @note        直接使用JPEG原始数据，跳过编解码步骤
 *              适用于纯图传场景，性能提升50-100倍
 ******************************************************************/
void CameraStreamServer::update_frame_jpeg(const std::vector<uchar>& jpeg_data)
{
    // 确保数据非空
    if (jpeg_data.empty()) {
        std::cerr << "错误: update_frame_jpeg() 接收到空数据" << std::endl;
        return;
    }

    uint64_t capture_ts_ms = now_ms();
    
    // 计算FPS
    static uint64_t last_capture_ts_local = 0;
    static double local_fps_estimate = 0.0;
    if (last_capture_ts_local != 0) {
        uint64_t delta = capture_ts_ms - last_capture_ts_local;
        if (delta > 0) {
            double instant_fps = 1000.0 / static_cast<double>(delta);
            if (local_fps_estimate <= 0.0) {
                local_fps_estimate = instant_fps;
            } else {
                local_fps_estimate = 0.85 * local_fps_estimate + 0.15 * instant_fps;
            }
            ema_fps = local_fps_estimate;
        }
    }
    last_capture_ts_local = capture_ts_ms;

    // 直接使用JPEG数据用于图传，无需重新编码
    latest_capture_ts_ms = capture_ts_ms;
    pthread_mutex_lock(&frame_mutex);

    // 如果容量过大，释放多余内存
    if (current_jpeg.capacity() > jpeg_data.size() * 2) {
        std::vector<unsigned char>().swap(current_jpeg);  // 完全释放
    }

    current_jpeg = jpeg_data;  // 直接赋值，零拷贝
    ++latest_frame_id;
    pthread_cond_broadcast(&frame_cond);
    pthread_mutex_unlock(&frame_mutex);
}

/*******************************************************************
 * @brief       停止摄像头图传服务器
 * 
 * @example     camera_server.stop_server();
 * 
 * @note        停止服务器并释放所有资源
 ******************************************************************/
void CameraStreamServer::stop_server(void)
{
    if (!running) return;
    
    std::cout << "正在停止摄像头图传服务器..." << std::endl;
    running = false;
    close_server_socket();
    pthread_cond_broadcast(&frame_cond);
    
    // 清空实例指针
    if (instance == this) {
        instance = nullptr;
    }
    
    std::cout << "摄像头图传服务器已停止" << std::endl;
}

/*******************************************************************
 * @brief       检查服务器是否正在运行
 * 
 * @return      返回服务器运行状态
 * @retval      true            服务器正在运行
 * @retval      false           服务器已停止
 * 
 * @example     if(camera_server.is_running()) {
 *                  //服务器正在运行
 *              }
 ******************************************************************/
bool CameraStreamServer::is_running(void)
{
    return running;
}
