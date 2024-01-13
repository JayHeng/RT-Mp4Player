# RT-Mp4Player

An MP4 player based on i.MXRT+emWin+FFmpeg3.0.1 | 一款基于i.MXRT的MP4播放器

### 1 硬件连接
#### 1.1 开发板
　　本应用基于的开发板是恩智浦 MIMXRT1170-EVKB，主芯片为 i.MXRT1062DVJ6A，主核 CM7 主频为 1GHz，内置 2MB SRAM（最大512KB TCM），开发板原理简图如下：  

![](https://raw.githubusercontent.com/JayHeng/pzhmcu-picture/master/github/RT-MPlayer_rt1170evkb_sch.PNG)  

　　本应用实际性能与板载存储资源性能息息相关，在此列出各存储资源性能表，后面具体分配资源时可作参考：  

<table><tbody>
    <tr>
        <th>模块</th>
        <th>CM7 Core</th>
        <th>I-cache</th>
        <th>D-cache</th>
        <th>ITCM</th>
        <th>DTCM</th>
        <th>OCRAM</th>
        <th>Hyper NOR</th>
        <th>SDRAM</th>
    </tr>
    <tr>
        <td>性能</td>
        <td>996MHz</td>
        <td>64bit@996MHz</td>
        <td>32bit@996MHz</td>
        <td>64bit@996MHz</td>
        <td>2x32bit@996MHz</td>
        <td>64bit@240MHz</td>
        <td>8bit@166MHz x 2(DDR)</td>
        <td>2x16bit@166MHz</td>
    </tr>
</table>

#### 1.2 SD卡
　　本应用需要配合SD卡使用，SD中存放多媒体源文件（如xx.mp4）。  

#### 1.3 LCD屏
　　本应用需要外接 MIPI DSI LCD 屏，用于显示视频/图片，可支持如下三种分辨率的 LCD 屏：  

##### 1.3.1 小屏 400*392
　　小屏即恩智浦官方 i.MX RT 系列 EVK 开发板配套的 1.2 寸 LCD 屏，具体型号为 G1120B0MIPI：  

> * NXP官网链接: https://www.nxp.com/design/design-center/development-boards/i-mx-evaluation-and-development-boards/1-2-wearable-display-g1120b0mipi:G1120B0MIPI  

　　该屏内置 GRAM，主控端 FrameBuffer 数据需要通过 DSI 发送到屏 GRAM 里，这里使用了 TE pin 来触发一帧刷完的中断，所以需要将开发板背面 R414 短接上。  

##### 1.3.2 中屏 960*540
　　中屏即恩智浦官方 i.MX RT 系列 EVK 开发板配套的 5.5 寸 LCD 屏，具体型号为 RK055IQH091：  

> * NXP官网链接: https://www.nxp.com/design/design-center/development-boards/i-mx-evaluation-and-development-boards/5-5-lcd-panel:RK055HDMIPI4M

　　该屏无内置 GRAM，主控端 FrameBuffer 数据通过 LCDIF 模块刷新。  

##### 1.3.3 大屏 1280*720
　　中屏即恩智浦官方 i.MX RT 系列 EVK 开发板配套的 5.5 寸 LCD 屏，具体型号为 RK055AHD091 或者 RK055MHD091：  

> * NXP官网链接: https://www.nxp.com/design/design-center/development-boards/i-mx-evaluation-and-development-boards/5-5-lcd-panel:RK055HDMIPI4MA0

　　该屏无内置 GRAM，主控端 FrameBuffer 数据通过 LCDIF 模块刷新。  

#### 1.4 音箱/耳机
　　本应用需要外接音箱/耳机，用于播放音频。  

### 2 程序使用
#### 2.1 准备mp4源
　　本应用测试的标准MP4源来自于 https://sample-videos.com/ 网站，该网站提供了动画电影Big Buck Bunny节选段的各种常见分辨率，比如1280x720，720x480，640x360等。但有时候我们需要对MP4做一些特殊的转换处理，虽然市面有很多UI软件（比如格式工厂、爱剪辑等），但最强大的还是命令行版本的ffmpeg。  

> * FFmpeg 4.1.1编译好共享版下载: https://ffmpeg.zeranoe.com/builds/win64/shared/ffmpeg-4.1.1-win64-shared.zip  

##### 2.1.1 转换分辨率
　　你可以下载1280x720分辨率版本（30MB）作为基础MP4，然后使用 \ffmpeg-4.1.1-win64-shared\bin\ffmpeg.exe工具对基础MP4进行处理：  

> jay@pc /d/ffmpeg-4.1.1-win64-shared/bin
> <font style="font-weight:bold;">.\ffmpeg.exe -i .\SampleVideo_1280x720_30mb.mp4 -vf scale=480:272 bunny_480x272.mp4 -hide_banner</font>

##### 2.1.2 转换采样率
　　音频采样率越低，MP4应用越流畅，因为留给软件解码H.264的时间越长：  

> jay@pc /d/ffmpeg-4.1.1-win64-shared/bin
> <font style="font-weight:bold;">.\ffmpeg.exe -i bunny_480x272.mp4 -ar 8000 -ac 2 bunny_480x272_8KHz.mp4</font>

##### 2.1.3 前移moov标志
　　当前应用仅能播放moov标志在mdat标志前面的MP4（与移植的嵌入式ffmpeg配置有关），因此需要借助ffmpeg.exe对源MP4进行处理：  

> jay@pc /d/ffmpeg-4.1.1-win64-shared/bin
> <font style="font-weight:bold;">.\ffmpeg.exe -i bunny_480x272_8KHz.mp4 -vcodec copy -acodec copy -movflags faststart bunny_480x272_faststart_8KHz.mp4</font>

#### 2.2 配置程序
　　本应用工程文件为 \RT-Mp4Player\boards\evkbmimxrt1170\demo_apps\sd_mp4\iar\mp4.eww，在测试下载本应用时需要注意以下两处配置：  

##### 2.2.1 各种linker下的工程
　　本应用在IAR v9.40.2 下编译占用存储资源如下：text（约640KB）、rodata（约240KB）、data & zi（约620KB）、NONCACHEABLE data（最大11MB，具体取决于LCD屏大小）、STACK（2MB）、HEAP（16MB，主要是ffmpeg需要，实测无法减小）。  

　　本应用基于的开发板存储资源为：内部FlexRAM（1MB）、内部OCRAM（1MB）、外部SDRAM（32+32MB）  

　　如果想测试不同存储策略下的性能，共有可以根据 linker file 创建多个 build：  

<table><tbody>
    <tr>
        <th rowspan="2">工程名</th>
        <th colspan="5">程序段位置</th>
    </tr>
    <tr>
        <td>**text,rodata**</td>
        <td>**data,bss**</td>
        <td>**NONCACHEABLE data**</td>
        <td>**STACK**</td>
        <td>**HEAP**</td>
    </tr>
    <tr>
        <td>**flexram_sdram**</td>
        <td>FlexRAM<br>-256KB ITCM<br>-256KB DTCM<br>-1.5MB OCRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
    </tr>
</table>

##### 2.2.2 一些重要的配置宏
　　工程重要的配置宏在 \RT-Mp4Player\boards\evkbmimxrt1170\demo_apps\sd_mp4\mp4.h中：  

```C
////////////////////////////////////////////////////////////////////////////////

// Set audio frame size according to source media
#define AUDIO_FRAME_SIZE   0x400
// Set audio cache frames, it is important for FFmpeg decode
#define AUDIO_CACHE_FRAMES 3
// Set audio buffer queue, it is important for SAI DMA transfer
#define AUDIO_BUFFER_QUEUE 3

// Set SAI configurations for audio
#define AUDIO_CONV_WIDTH   kSAI_WordWidth16bits

////////////////////////////////////////////////////////////////////////////////

// Set PXP conversation method for video
#define VIDEO_PXP_CONV_BLOCKING       1
#define VIDEO_PXP_CONV_WAITING        1
// Set LCD display method for video
#define VIDEO_LCD_DISP_BLOCKING       1
#define VIDEO_LCD_DISP_WAITING        1

// Set LCD resolution
#define VIDEO_LCD_RESOLUTION_ROUND400 0  // For 400x392 LCD
#define VIDEO_LCD_RESOLUTION_SVGA540  1  // For 960*540 LCD
#define VIDEO_LCD_RESOLUTION_WXGA720  0  // For 1280*720 LCD

// Set PXP converted pixel format
#define VIDEO_PIXEL_FMT_RGB888        0
#define VIDEO_PIXEL_FMT_RGB565        1
```

### 3 其他功能
#### 3.1 统计FFmpeg解码时间
　　在 \RT-Mp4Player\boards\evbkmimxrt1170\demo_apps\sd_mp4\mp4.h 中开启下面2个宏，编译应用工程执行后会在SD卡中生成time.txt文件，该文件保存了ffmpeg解码以及PXP,LCD耗时数据（默认是统计2000帧音视频数据，如需修改请在工程中搜索 FF_MEASURE_FRAMES）。  

```C
// Set to measure FFmpeg decode time for each frame
#define MP4_FF_TIME_ENABLE  0

// Set to measure LCD display time for each frame
// !!!Note: it can be enabled along with MP4_FF_TIME_ENABLE
#define MP4_LCD_TIME_ENABLE 0
```

　　使用 \RT-Mp4Player\bin\count_ffmpeg_time.py 脚本可以读取time.txt中的数据并用图表显示出来，脚本命令如下（其中xxResoultion, xxBuild参数仅是用于在图表中显示标题信息）：  

> jay@pc /d/RT-Mp4Player/bin
> <font style="font-weight:bold;">python .\count_ffmpeg_time.py .\time.txt 'xxResolution' 'xxBuild'</font>

　　下图为脚本执行结果，共显示4张表，分别是FFmpeg读帧数据时间、解音频帧时间、解视频帧时间、PXP转换视频帧时间（YUV444转RGB565）。  

![](https://raw.githubusercontent.com/JayHeng/pzhmcu-picture/master/github/RT-MPlayer_ffmpeg_time_400x400_flexram_sdram.PNG)  

　　如下是 flexram_sdram 工程在不同源 MP4 分辨率以及不同 LCD 屏上实测解码帧率表：  

<table><tbody>
    <tr>
        <th rowspan="2">源MP4分辨率</th>
        <th colspan="3">RT1170在LCD屏上解码帧率（fps）</th>
    </tr>
    <tr>
        <td>ROUND400（400x392）</td>
        <td>SVGA540（960x540）</td>
        <td>WXGA720（1280x720）</td>
    </tr>
    <tr>
        <td>TGA120（192x120）</td>
        <td>140.38</td>
        <td>76.8</td>
        <td>56.91</td>
    </tr>
    <tr>
        <td>MGA180（288x180）</td>
        <td>87.82</td>
        <td>55.85</td>
        <td>47.67</td>
    </tr>
    <tr>
        <td>CGA240（320x240）</td>
        <td>79.13</td>
        <td>50.83</td>
        <td>41.64</td>
    </tr>
    <tr>
        <td>HVGA272（480x272）</td>
        <td>46.3</td>
        <td>34.71</td>
        <td>29.14</td>
    </tr>
    <tr>
        <td>ROUND400（400x400）</td>
        <td>38.6</td>
        <td>28.45</td>
        <td>TBD</td>
    </tr>
    <tr>
        <td>SVGA540（960x540）</td>
        <td>/</td>
        <td>10.82</td>
        <td>10</td>
    </tr>
    <tr>
        <td>WXGA720（1280x720）</td>
        <td>/</td>
        <td>/</td>
        <td>8.85</td>
    </tr>
</table>
