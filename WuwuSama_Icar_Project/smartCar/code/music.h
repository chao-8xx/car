#ifndef __MUSCI_H_
#define __MUSCI_H_
#include "headfile.h"

// 音符结构体
typedef struct {
    uint32_t frequency;
    uint32_t duration;
} Note;

extern const Note qing_tian[243];

void play_music(const Note *song, uint32_t length);

// ==========================================
// 异步音乐播放类 (新增)
// ==========================================
class AsyncMusicPlayer {
public:
    AsyncMusicPlayer();
    ~AsyncMusicPlayer();

    // 异步播放音乐
    void play(const Note* song, uint32_t length);
    
    // 停止播放
    void stop();
    
    // 是否正在播放
    bool is_playing() const;

private:
    // 播放线程函数
    void worker_func();
    
    // 播放单个音符（支持中断）
    void play_note_interruptible(uint32_t frequency, uint32_t duration_ms);

    std::thread worker;              // 后台线程
    std::atomic<bool> running;       // 线程运行标志
    std::atomic<bool> should_play;   // 是否应该播放
    std::atomic<bool> stop_requested;// 请求停止标志
    
    const Note* current_song;        // 当前歌曲指针
    uint32_t current_song_len;       // 当前歌曲长度

    std::mutex mtx;
    std::condition_variable cv;
};

extern AsyncMusicPlayer music_player;

#endif
