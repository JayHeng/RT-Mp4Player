# RT-Mp4Player

An MP4 player based on i.MXRT+emWin+FFmpeg3.0.1 | 一款基于i.MXRT的MP4播放器

### 1 硬件连接
#### 1.1 开发板
　　本应用基于的开发板是恩智浦 MIMXRT1060EVK12，主芯片为i.MXRT1062DVJ6A，主频为600MHz，内置1MB SRAM（最大512KB TCM），开发板原理简图如下：  

<img src="http://henjay724.com/image/github/RT-MPlayer_rt1060evk12_sch.PNG" style="zoom:100%" />  

　　本应用实际性能与板载存储资源性能息息相关，在此列出各存储资源性能表，后面具体分配资源时可作参考：  

<table><tbody>
    <tr>
        <th>模块</th>
        <th>Core</th>
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
        <td>600MHz</td>
        <td>64bit@600MHz</td>
        <td>32bit@600MHz</td>
        <td>64bit@600MHz</td>
        <td>2x32bit@600MHz</td>
        <td>64bit@133MHz</td>
        <td>8bit@166MHz x 2(DDR)</td>
        <td>16bit@166MHz</td>
    </tr>
</table>

#### 1.2 SD卡
　　本应用需要配合SD卡使用，SD中存放多媒体源文件（如xx.mp4）。  

#### 1.3 LCD屏
　　本应用需要外接LCD屏，用于显示视频/图片，可支持如下三种分辨率的LCD屏：  

##### 1.3.1 小屏 480*272
　　小屏即恩智浦官方i.MX RT系列EVK开发板配套的4.3寸LCD屏，具体型号为 RK043FN02H-CT：  

> * NXP官网链接: https://www.nxp.com/support/developer-resources/software-development-tools/i.mx-developer-resources/4.3-lcd-panel:RK043FN02H-CT  
> * Rocktech官网链接: http://www.rocktech.com.hk/news_view.aspx?TypeId=5&Id=393&FId=t2:5:2  

　　该屏与开发板连接图如下：  

<img src="http://henjay724.com/image/github/RT-MPlayer_LCD_480x272.jpg" style="zoom:100%" />  

##### 1.3.2 中屏 800*600
　　中屏是恩智浦合作伙伴ZLG开发的一款8寸LCD屏，具体型号为 LCD-800600W080TR：  

> * ZLG官网链接: http://www.zlg.cn/Ipc/Ipc/parts  

　　该屏与开发板连接图如下（开发板上电阻R67需要移除，直连即可）。注意该屏必须同时连接6pin触摸线，否则显示不正常。  

<img src="http://henjay724.com/image/github/RT-MPlayer_LCD_800x600.jpg" style="zoom:100%" />  

##### 1.3.3 大屏 1280*800
　　大屏为型号未知的10.1寸LCD屏：  

　　该屏与开发板连接图如下（开发板上电阻R67需要移除，直连即可）：  

<img src="http://henjay724.com/image/github/RT-MPlayer_LCD_1280x800.jpg" style="zoom:100%" />  

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
　　本应用工程文件为 \RT-Mp4Player\boards\evkmimxrt1060\demo_apps\sd_mp4\iar\mp4.eww，在测试下载本应用时需要注意以下两处配置：  

##### 2.2.1 各种linker下的工程
　　本应用在IAR v8.32.2下编译占用存储资源如下：text（约640KB）、rodata（约240KB）、data & zi（约620KB）、NONCACHEABLE data（最大11MB，具体取决于LCD屏大小）、STACK（2MB）、HEAP（16MB，主要是ffmpeg需要，实测无法减小）。  
　　本应用基于的开发板存储资源为：内部FlexRAM（1MB）、外部SDRAM（32MB）、外部Hyper NOR Flash（64MB），为了测试不同存储策略下的性能，共有4种不同linker版本的工程：  

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
        <td>**sdram**</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
    </tr>
    <tr>
        <td>**flexram_sdram**</td>
        <td>FlexRAM<br>-128KB ITCM<br>-128KB DTCM<br>-786KB OCRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
    </tr>
    <tr>
        <td>**flexspinor_sdram**</td>
        <td>Hyper NOR Flash</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
    </tr>
    <tr>
        <td>**flexspinor_flexram_sdram**</td>
        <td>Hyper NOR Flash</td>
        <td>FlexRAM<br>-512KB ITCM<br>-512KB OCRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
        <td>SDRAM</td>
    </tr>
</table>

##### 2.2.2 一些重要的配置宏
　　工程重要的配置宏在 \RT-Mp4Player\boards\evkmimxrt1060\demo_apps\sd_mp4\mp4.h中：  

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
#define VIDEO_LCD_DISP_WAITING        0

// Set LCD resolution
#define VIDEO_LCD_RESOLUTION_HVGA272  1  // For 480*272 LCD
#define VIDEO_LCD_RESOLUTION_SVGA600  0  // For 800*600 LCD
#define VIDEO_LCD_RESOLUTION_WXGA800  0  // For 1280*800 LCD

// Set PXP converted pixel format
#define VIDEO_PIXEL_FMT_RGB888        0
#define VIDEO_PIXEL_FMT_RGB565        1

// Set LCD refresh frequency
#define VIDEO_LCD_REFRESH_FREG_60Hz   0
#define VIDEO_LCD_REFRESH_FREG_30Hz   0
#define VIDEO_LCD_REFRESH_FREG_25Hz   1
```

#### 2.3 下载程序
##### 2.3.1 使用外部J-Link
　　推荐使用J-Link下载sdram和flexram_sdram工程，其中对于SDRAM的初始化配置由 \RT-Mp4Player\boards\evkmimxrt1060\demo_apps\sd_mp4\evkmimxrt1060_sdram_init_166mhz.jlinkscript 脚本完成。  

<img src="http://henjay724.com/image/github/RT-MPlayer_config_sdram_jlink.PNG" style="zoom:100%" />  

##### 2.3.2 使用板载CMSIS-DAP
　　推荐使用CMSIS-DAP下载flexspinor_xxx工程（IAR v8.32.2 Flashloader不支持RT1060EVK Hyper NOR Flash），其中对于SDRAM的初始化配置由 \RT-Mp4Player\boards\evkmimxrt1060\demo_apps\sd_mp4\iar\evkmimxrt1060_sdram_init_166mhz.mac 脚本完成。  

<img src="http://henjay724.com/image/github/RT-MPlayer_config_sdram_cmsisdap.PNG" style="zoom:100%" />  

##### 2.3.3 XIP启动配置
　　XIP启动需要配置FlexSPI NOR Flash和SDRAM，这是由ROM来实现的，应用工程里如下两个源文件里指定了配置参数：  

> FlexSPI NOR配置: \RT-Mp4Player\boards\evkmimxrt1060\xip\evkmimxrt1060_hyper_nor_config_166mhz.c  
> SDRAM配置: \RT-Mp4Player\boards\evkmimxrt1060\xip\evkmimxrt1060_sdram_ini_dcd_166mhz.c  

### 3 其他功能
#### 3.1 统计FFmpeg解码时间
　　在 \RT-Mp4Player\boards\evkmimxrt1060\demo_apps\sd_mp4\mp4.h 中开启下面2个宏，编译应用工程执行后会在SD卡中生成time.txt文件，该文件保存了ffmpeg解码以及PXP,LCD耗时数据（默认是统计2000帧音视频数据，如需修改请在工程中搜索 FF_MEASURE_FRAMES）。  

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

<img src="http://henjay724.com/image/github/RT-MPlayer_ffmpeg_time_hvga272_to_hvga272_flexram_sdram.PNG" style="zoom:100%" />  

　　如下是flexram_sdram工程在不同源MP4分辨率以及不同LCD屏上实测解码帧率表：  

<table><tbody>
    <tr>
        <th rowspan="2">源MP4分辨率</th>
        <th colspan="3">在LCD屏上解码帧率（fps）</th>
    </tr>
    <tr>
        <td>**HVGA272（480x272）**</td>
        <td>**SVGA600（800x600）**</td>
        <td>**WXGA800（1280x800）**</td>
    </tr>
    <tr>
        <td>**TGA120（192x120）**</td>
        <td>82.79</td>
        <td>43.52</td>
        <td>23.44</td>
    </tr>
    <tr>
        <td>**MGA180（288x180）**</td>
        <td>53.62</td>
        <td>33.31</td>
        <td>19.94</td>
    </tr>
    <tr>
        <td>**CGA240（320x240）**</td>
        <td>41.73</td>
        <td>27.76</td>
        <td>17.46</td>
    </tr>
    <tr>
        <td>**HVGA272（480x272）**</td>
        <td>29.26</td>
        <td>21.2</td>
        <td>14.7</td>
    </tr>
    <tr>
        <td>**SVGA600（800x600）**</td>
        <td>/</td>
        <td>8.13</td>
        <td>6.56</td>
    </tr>
    <tr>
        <td>**WXGA800（1280x800）**</td>
        <td>/</td>
        <td>/</td>
        <td>3.83</td>
    </tr>
</table>
