#ifndef CSCREENRECORDER_H
#define CSCREENRECORDER_H
extern "C"
{
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libavdevice/avdevice.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/imgutils.h"
#include "libswresample/swresample.h"
#include "libavutil/avassert.h"
}

#include <QString>
#include <condition_variable>
#include <QThread>
#include <thread>
#include <mutex>
#include <atomic>
#include <QVariantMap>
#include <QDebug>
#include <QObject>
#include <QImage>

#define cout qDebug() << __FUNCTION__ << __LINE__

const int32_t OK = {0};
const int32_t ERR = {1};
enum class  EnRecordState
{
    EN_START,
    EN_PAUSE,
    EN_STOP
};
class CScreenRecorder
{
public:
    explicit CScreenRecorder();
    void Init(const QVariantMap& _map);
    void Start();
    void Pause();
    void Stop();

private:
    void muxThread();
    void screenRecordThread();
    void soundRecordThread();

    int openVideo();
    int openAudio();
    int openOutput();

    void initVideoBuffer();
    void initAudioBuffer();
    void flushVideoDecoder();
    void flushAudioDecoder();
    void flushEncoders();

    void release();
    int check_sample_fmt(const AVCodec *codec, enum AVSampleFormat sample_fmt);
    AVFrame* allocAudioFrame(AVCodecContext* _pEncodec, int _nbSamples);
    QString getAvErrorMsg(const int& _nErr);

private:
    QString             m_strMicrophoneName;
    QString				m_strFilePath;
    int					m_nWidth            = {0};
    int					m_nHeight           = {0};
    int					m_nFps              = {0};
    int					m_nAudioBitrate     = {0};
    int                 m_nVideoIndex       = {0};
    int                 m_nAudioIndex       = {0};
    int                 m_nVideoOutIndex    = {0};
    int                 m_nAudioOutIndex    = {0};
    int					m_nbSamples         = {0};
    int64_t				m_nVideoCurPts      = {0};
    int64_t				m_nAudioCurPts      = {0};

    AVFormatContext		*m_pVideoFmtCtx    = {nullptr};
    AVFormatContext		*m_pAudioFmtCtx    = {nullptr};
    AVFormatContext		*m_pOutFmtCtx      = {nullptr};
    AVCodecContext		*m_pVideoDecodeCtx = {nullptr};
    AVCodecContext		*m_pAudioDecodeCtx = {nullptr};
    AVCodecContext		*m_pVideoEncodeCtx = {nullptr};
    AVCodecContext		*m_pAudioEncodeCtx = {nullptr};

    SwsContext			*m_pSwsCtx       = {nullptr};
    SwrContext			*m_pSwrCtx       = {nullptr};
    AVFifoBuffer		*m_pVideoFifoBuf = {nullptr};
    AVAudioFifo			*m_pAudioFifoBuf = {nullptr};

    AVFrame				*m_pVideoOutFrame    = {nullptr};
    uint8_t				*m_pVideoOutFrameBuf = {nullptr};
    int					m_nVideoOutFrameSize = {0};


    std::atomic<EnRecordState> m_enState = {EnRecordState::EN_STOP};
    std::condition_variable m_cvNotPause;	//当点击暂停的时候，两个采集线程挂起
    std::mutex				m_mtxPause;

    std::condition_variable m_cvVBufNotFull;
    std::condition_variable m_cvVBufNotEmpty;
    std::mutex				m_mtxVBuf;

    std::condition_variable m_cvABufNotFull;
    std::condition_variable m_cvABufNotEmpty;
    std::mutex				m_mtxABuf;


};

#endif // CSCREENRECORDER_H
