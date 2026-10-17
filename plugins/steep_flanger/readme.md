# steep flanger
steep flanger is a FIR/IIR flanger plugin.

## features
FIR flanger  
IIR flanger  
barberpole flanging  

## gui
![GUI](docs/gui.png)

## demo
[bilibili](https://www.bilibili.com/video/BV198npzXEPn)  
[youtube](https://www.youtube.com/watch?v=yQ357S2LP2w)  

## Install Plugin / 安装插件

从Release下载对应操作系统的压缩包，解压后大致会看到这样的结构  
Download the compressed package for the corresponding operating system from Release. After extracting it, you will generally see a structure like this:  

```text
pluginName-win-vX.Y.Z.zip
  VST3/
    PluginName.vst3/

pluginName-macos-vX.Y.Z.zip
  AU/
    PluginName.component/
  VST3/
    PluginName.vst3/

pluginName-linux-vX.Y.Z.zip
  LV2/
    PluginName.lv2/
  VST3/
    PluginName.vst3/
```

安装时请直接复制这些文件夹之一：  
When installing, copy one of these folders directly:

- `PluginName.vst3`
- `PluginName.component`
- `PluginName.lv2`

常见安装目录：  
Common install locations:

- Windows VST3: `C:\Program Files\Common Files\VST3\`
- macOS VST3: `/Library/Audio/Plug-Ins/VST3/` or `~/Library/Audio/Plug-Ins/VST3/`
- macOS AU: `/Library/Audio/Plug-Ins/Components/` or `~/Library/Audio/Plug-Ins/Components/`
- Linux LV2: `~/.lv2/` or `/usr/lib/lv2/`
- Linux VST3: `~/.vst3/` or `/usr/lib/vst3/`

例如在 Windows 上，不要复制 `VST3` 文件夹本身，而是把其中的 `PluginName.vst3` 整个文件夹复制到 `C:\Program Files\Common Files\VST3\`。  
For example, on Windows, do not copy the `VST3` folder itself. Copy the whole `PluginName.vst3` folder inside it to `C:\Program Files\Common Files\VST3\`.

额外的，macOS 用户还需要做以下工作：  
Additionally, macOS users may need to do the following:

```bash
sudo xattr -dr com.apple.quarantine /path/to/your/plugins/plugin_name.component
sudo xattr -dr com.apple.quarantine /path/to/your/plugins/plugin_name.vst3
sudo xattr -dr com.apple.quarantine /path/to/your/plugins/plugin_name.lv2
```

如果 macOS 阻止打开下载的插件，可以对插件 bundle 执行上面的命令来移除 quarantine 属性。  
If macOS blocks a downloaded plugin, you can run the commands above on the plugin bundle to remove the quarantine attribute.

## signal-flow
### FIR PART ###
![alt text](docs/image.png)

### IIR PART ###
TODO

## 构建(Build)

```bash
git clone --recurse https://github.com/ManasWolrd/SteepFlanger.git

# Windows
cmake -G "Ninja" -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang -DCMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build --config Release

# Linux
sudo apt update
sudo apt-get install libx11-dev libfreetype-dev libfontconfig1-dev libasound2-dev libxrandr-dev libxinerama-dev libxcursor-dev
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build --config Release

# macOS
cmake -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES="x86_64;arm64" -S . -B build
cmake --build build --config Release
```
