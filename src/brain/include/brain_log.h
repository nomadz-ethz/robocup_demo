#pragma once

#include <string>
#include <rerun.hpp>

class Brain; // 向前声明

using namespace std;

/**
 * rerun log 相关的操作放到这个库里进行
 * 如果有些重复性的日志需要打印，可以把函数封装在这个类里
 */
class BrainLog
{
public:
    /**
     * 构造函数里只初始化对象，并设置 enabled 为 false
     */
    BrainLog(Brain *argBrain);

    // 往 rerun log 输出静态日志, 如场地标线等.
    void prepare();

    // 向 rerun log 写入静态内容, 如球场标线等
    void logStatics();

    // 设置当前时间
    void setTimeNow();

    // 手动设置时间
    void setTimeSeconds(double seconds);

    // 暴露 rerun::RecordingStream 一样的接口
    template <typename... Ts>
    inline void log(string_view entity_path, const Ts &...archetypes_or_collections) const
    {
        if (enabled)
            rerunLog.log(entity_path, archetypes_or_collections...);
    }

    // 在 img log 上直观地展示信息. 会在 img 外 padding 像素外画一个 color 颜色的框, 并在下方标注 text 文字
    void logToScreen(string logPath, string text, u_int32_t color, double padding = 0.0);

    inline bool isEnabled() {
        return enabled;
    }
private:
    bool enabled;
    Brain *brain;
    rerun::RecordingStream rerunLog;
};