#include "cscreenrecorder.h"
#include <QPainter>
CScreenRecorder::CScreenRecorder()
{

}

void CScreenRecorder::Init(const QVariantMap &_map)
{
    m_strFilePath = _map["filePath"].toString();
    m_nWidth = _map["width"].toInt();
    m_nHeight = _map["height"].toInt();
    m_nFps = _map["fps"].toInt();
    m_nAudioBitrate = _map["audioBitrate"].toInt();

    m_strMicrophoneName = _map["MicrophoneName"].toString();
}

void CScreenRecorder::Start()
{
    if (m_enState == EnRecordState::EN_STOP)
    {
        cout << "start record";
        m_enState = EnRecordState::EN_START;
        std::thread muxThread(&CScreenRecorder::muxThread, this);
        muxThread.detach();
    }
    else if (m_enState == EnRecordState::EN_PAUSE)
    {
        cout << "continue record";
        m_enState = EnRecordState::EN_START;
        m_cvNotPause.notify_one();
    }
}

void CScreenRecorder::Pause()
{
    m_enState = EnRecordState::EN_PAUSE;
}

void CScreenRecorder::Stop()
{
    EnRecordState state = m_enState;
    m_enState = EnRecordState::EN_STOP;
    if (state == EnRecordState::EN_PAUSE)
        m_cvNotPause.notify_one();
}

void CScreenRecorder::muxThread()
{
    cout << " start";
    int nRet = -1;
    bool bFinish = false;
    int nVideoFrameIndex = 0;
    int nAudioFrameIndex = 0;

    avdevice_register_all();
    if( openVideo() == ERR ||openAudio() == ERR || openOutput() == ERR)
    {
        release();
        cout << " open device failed, exit...";
        return;
    }
    initVideoBuffer();
    initAudioBuffer();
    std::thread screenRecord(&CScreenRecorder::screenRecordThread, this);
    std::thread soundRecord(&CScreenRecorder::soundRecordThread, this);
    screenRecord.detach();
    soundRecord.detach();
    QThread::msleep(200);

    m_nVideoCurPts = m_nAudioCurPts = 0;
    while(true)
    {
        AVPacket pkt;
        av_init_packet(&pkt);
        if(m_enState == EnRecordState::EN_STOP && !bFinish)
            bFinish = true;
        if(bFinish)
        {
            std::unique_lock<std::mutex> vBufLock(m_mtxVBuf, std::defer_lock);
            std::unique_lock<std::mutex> aBufLock(m_mtxABuf, std::defer_lock);
            std::lock(vBufLock, aBufLock);
            if (av_fifo_size(m_pVideoFifoBuf) < m_nVideoOutFrameSize &&
                av_audio_fifo_size(m_pAudioFifoBuf) < m_nbSamples )
            {
                cout << "video buff size:" << av_fifo_size(m_pVideoFifoBuf) << " audio buff size:" << av_audio_fifo_size(m_pAudioFifoBuf) << " both video and audio fifo buf are empty, break";
                break;
            }
            cout << "video buff size:" << av_fifo_size(m_pVideoFifoBuf) << " audio buff size:" << av_audio_fifo_size(m_pAudioFifoBuf);
            if(av_fifo_size(m_pVideoFifoBuf) >= m_nVideoOutFrameSize && m_nVideoCurPts == INT_MAX)
                m_nVideoCurPts = 0;
            if(av_audio_fifo_size(m_pAudioFifoBuf) >= m_nbSamples && m_nAudioCurPts == INT_MAX)
                m_nAudioCurPts = 0;
        }
        // 比较音视频pts，大于0表示视频帧在前，音频需要连续编码。
        // 小于0表示，音频帧在前，应该至少编码一帧视频
        if (av_compare_ts(m_nVideoCurPts, m_pOutFmtCtx->streams[m_nVideoOutIndex]->time_base,
            m_nAudioCurPts, m_pOutFmtCtx->streams[m_nAudioOutIndex]->time_base) <= 0)
        {
            if (bFinish)
            {
                std::lock_guard<std::mutex> lk(m_mtxVBuf);
                if (av_fifo_size(m_pVideoFifoBuf) < m_nVideoOutFrameSize)
                {
                    cout << "video wirte done";
                    m_nVideoCurPts = INT_MAX;
                    continue;
                }
            }
            else
            {
                std::unique_lock<std::mutex> lk(m_mtxVBuf);
                m_cvVBufNotEmpty.wait(lk, [this] { return av_fifo_size(m_pVideoFifoBuf) >= m_nVideoOutFrameSize; });
            }

            av_fifo_generic_read(m_pVideoFifoBuf, m_pVideoOutFrameBuf, m_nVideoOutFrameSize, nullptr);
            m_cvVBufNotFull.notify_one();

            m_pVideoOutFrame->pts = nVideoFrameIndex++;
            m_pVideoOutFrame->format = m_pVideoEncodeCtx->pix_fmt;
            m_pVideoOutFrame->width = m_pVideoEncodeCtx->width;
            m_pVideoOutFrame->height = m_pVideoEncodeCtx->height;

            nRet = avcodec_send_frame(m_pVideoEncodeCtx, m_pVideoOutFrame);
            if(nRet != OK)
            {
                cout << "video avcodec_send_frame failed:" <<  getAvErrorMsg(nRet);
                continue;
            }

            nRet = avcodec_receive_packet(m_pVideoEncodeCtx, &pkt);
            if(nRet != OK)
            {
                cout << "video avcodec_receive_packet failed:" << getAvErrorMsg(nRet);
                av_packet_unref(&pkt);
                continue;
            }
            pkt.stream_index = m_nVideoIndex;
            av_packet_rescale_ts(&pkt, m_pVideoEncodeCtx->time_base, m_pOutFmtCtx->streams[m_nVideoOutIndex]->time_base);

            m_nVideoCurPts = pkt.pts;

            nRet = av_interleaved_write_frame(m_pOutFmtCtx, &pkt);
            if(nRet != OK)
            {
                cout << "video av_interleaved_write_frame failed:" << nRet;
            }
            cout << "video index:" << nVideoFrameIndex;
            av_packet_unref(&pkt);
        }
        else
        {
            if (bFinish)
            {
                std::lock_guard<std::mutex> lk(m_mtxABuf);
                if (av_audio_fifo_size(m_pAudioFifoBuf) < m_nbSamples)
                {
                    cout << "audio write done";
                    m_nAudioCurPts = INT_MAX;
                    continue;
                }
            }
            else
            {
                std::unique_lock<std::mutex> lk(m_mtxABuf);
                m_cvABufNotEmpty.wait(lk, [this] { return av_audio_fifo_size(m_pAudioFifoBuf) >= m_nbSamples; });
            }
            int nRet = -1;
            AVFrame *pFrame = av_frame_alloc();
            pFrame->nb_samples = m_nbSamples;
            pFrame->channel_layout = m_pAudioEncodeCtx->channel_layout;
            pFrame->format = m_pAudioEncodeCtx->sample_fmt;
            pFrame->sample_rate = m_pAudioEncodeCtx->sample_rate;
            pFrame->pts = m_nbSamples * nAudioFrameIndex++;
            //分配data buf
            nRet = av_frame_get_buffer(pFrame, 0);
            if(av_audio_fifo_read(m_pAudioFifoBuf, (void **)pFrame->data, m_nbSamples) < m_nbSamples)
            {
                cout << "av_audio_fifo_read failed";
                continue;
            }
            m_cvABufNotFull.notify_one();

            nRet = avcodec_send_frame(m_pAudioEncodeCtx, pFrame);
            if (nRet != OK)
            {
                cout << "audio avcodec_send_frame failed, ret: " << nRet;
                av_frame_free(&pFrame);
                continue;
            }
            nRet = avcodec_receive_packet(m_pAudioEncodeCtx, &pkt);
            if (nRet != OK)
            {
                cout << "audio avcodec_receive_packet failed, ret: " << nRet;
                av_frame_free(&pFrame);
                av_packet_unref(&pkt);
                continue;
            }
            pkt.stream_index = m_nAudioOutIndex;

            av_packet_rescale_ts(&pkt, m_pAudioEncodeCtx->time_base, m_pOutFmtCtx->streams[m_nAudioOutIndex]->time_base);

            m_nAudioCurPts = pkt.pts;

            nRet = av_interleaved_write_frame(m_pOutFmtCtx, &pkt);
            if (nRet != OK)
                cout << "audio av_interleaved_write_frame failed, ret: " << nRet;

            cout << "audio index:" << nAudioFrameIndex;
            av_frame_free(&pFrame);
            av_packet_unref(&pkt);
        }
    }

    flushEncoders();
    av_write_trailer(m_pOutFmtCtx);

    cout << " exit";
    release();
}

void CScreenRecorder::screenRecordThread()
{
    cout << " start";
    int nRet = -1;
    AVPacket pkt;
    av_init_packet(&pkt);

    int y_size = m_nWidth * m_nHeight;
    AVFrame	*pRawFrame = av_frame_alloc();
    AVFrame *pNewFrame = av_frame_alloc();

    int newFrameBufSize = av_image_get_buffer_size(m_pVideoEncodeCtx->pix_fmt, m_nWidth, m_nHeight, 1);
    uint8_t *pNewFrameBuf = (uint8_t*)av_malloc(newFrameBufSize);
    av_image_fill_arrays(pNewFrame->data, pNewFrame->linesize, pNewFrameBuf,
        m_pVideoEncodeCtx->pix_fmt, m_nWidth, m_nHeight, 1);
    while(m_enState != EnRecordState::EN_STOP)
    {
        if (m_enState == EnRecordState::EN_PAUSE)
        {
            std::unique_lock<std::mutex> lk(m_mtxPause);
            m_cvNotPause.wait(lk, [this] { return m_enState != EnRecordState::EN_PAUSE; });
        }
        nRet = av_read_frame(m_pVideoFmtCtx, &pkt);
        if(nRet < OK)
        {
            cout << "video read frame failed:" << nRet;
            continue;
        }

        if(pkt.stream_index != m_nVideoIndex)
        {
            cout << "not video stream index";
            av_packet_unref(&pkt);
            continue;
        }

        nRet = avcodec_send_packet(m_pVideoDecodeCtx, &pkt);
        if(nRet != OK)
        {
            cout << "video avcodec_send_packet failed:" << nRet;
            continue;
        }

        nRet = avcodec_receive_frame(m_pVideoDecodeCtx, pRawFrame);
        if(nRet != OK)
        {
            cout << "video avcodec_receive_frame failed:" << nRet;
            continue;
        }

        sws_scale(m_pSwsCtx, (const uint8_t* const*)pRawFrame->data, pRawFrame->linesize, 0,
            m_pVideoEncodeCtx->height, pNewFrame->data, pNewFrame->linesize);

        {
            std::unique_lock<std::mutex> lk(m_mtxVBuf);
            m_cvVBufNotFull.wait(lk, [this] { return av_fifo_space(m_pVideoFifoBuf) >= m_nVideoOutFrameSize; });
        }
        av_fifo_generic_write(m_pVideoFifoBuf, pNewFrame->data[0], y_size, nullptr);
        av_fifo_generic_write(m_pVideoFifoBuf, pNewFrame->data[1], y_size / 4, nullptr);
        av_fifo_generic_write(m_pVideoFifoBuf, pNewFrame->data[2], y_size / 4, nullptr);
        m_cvVBufNotEmpty.notify_one();

        av_packet_unref(&pkt);
    }

    flushVideoDecoder();

    av_free(pNewFrameBuf);
    av_frame_free(&pRawFrame);
    av_frame_free(&pNewFrame);
    cout << "screen record thread exit";
}

void CScreenRecorder::soundRecordThread()
{
    cout << " start";
    int nRet = -1;
    AVPacket pkt;
    av_init_packet(&pkt);
    int nbSamples = m_nbSamples;
    int nDstNbSamples, nnMaxDstNbSamples;
    AVFrame *pRawFrame = av_frame_alloc();
    AVFrame *pNewFrame = allocAudioFrame(m_pAudioEncodeCtx, nbSamples);

    nnMaxDstNbSamples = nDstNbSamples = av_rescale_rnd(nbSamples,
        m_pAudioEncodeCtx->sample_rate, m_pAudioDecodeCtx->sample_rate, AV_ROUND_UP);

    while(m_enState != EnRecordState::EN_STOP)
    {
        nRet = av_read_frame(m_pAudioFmtCtx, &pkt);
        if(nRet < OK)
        {
            cout << "audio av_read_frame faild:" << nRet;
            continue;
        }
        if(pkt.stream_index != m_nAudioIndex)
        {
            av_packet_unref(&pkt);
            cout << "is not audio stream index";
            continue;
        }
        nRet = avcodec_send_packet(m_pAudioDecodeCtx, &pkt);
        if(nRet != OK)
        {
            cout << "audio avcodec_send_packet failed:" << nRet;
            continue;
        }

        nRet = avcodec_receive_frame(m_pAudioDecodeCtx, pRawFrame);
        if(nRet != OK)
        {
            cout << "audio avcodec_receive_frame failed:" << nRet;
            continue;
        }

        nDstNbSamples = av_rescale_rnd(swr_get_delay(m_pSwrCtx, m_pAudioDecodeCtx->sample_rate) + pRawFrame->nb_samples,
            m_pAudioEncodeCtx->sample_rate, m_pAudioDecodeCtx->sample_rate, AV_ROUND_UP);
        if (nDstNbSamples > nnMaxDstNbSamples)
        {
            cout << "audio newFrame realloc";
            av_freep(&pNewFrame->data[0]);
            //nb_samples*nb_channels*Bytes_sample_fmt
            nRet = av_samples_alloc(pNewFrame->data, pNewFrame->linesize, m_pAudioEncodeCtx->channels,
                nDstNbSamples, m_pAudioEncodeCtx->sample_fmt, 1);
            if (nRet < OK)
            {
                cout << "av_samples_alloc failed";
                return;
            }

            nnMaxDstNbSamples = nDstNbSamples;
            m_pAudioEncodeCtx->frame_size = nDstNbSamples;
            m_nbSamples = pNewFrame->nb_samples;	//1024

        }
        pNewFrame->nb_samples = swr_convert(m_pSwrCtx, pNewFrame->data, nDstNbSamples,
                    (const uint8_t**)pRawFrame->data, pRawFrame->nb_samples);
        if(pNewFrame->nb_samples < OK)
        {
            cout << "audio swr_convert failed";
            return;
        }
        {
            std::unique_lock<std::mutex> lk(m_mtxABuf);
            m_cvABufNotFull.wait(lk, [pNewFrame, this] { return av_audio_fifo_space(m_pAudioFifoBuf) >= pNewFrame->nb_samples; });
        }
        if (av_audio_fifo_write(m_pAudioFifoBuf, (void **)pNewFrame->data, pNewFrame->nb_samples) < pNewFrame->nb_samples)
        {
            cout << "av_audio_fifo_write";
            return;
        }
        m_cvABufNotEmpty.notify_one();
    }

    flushAudioDecoder();
    av_frame_free(&pRawFrame);
    av_frame_free(&pNewFrame);
    cout<< "exit";
}

int CScreenRecorder::openVideo()
{
    AVInputFormat*fmt = av_find_input_format("gdigrab");
    AVDictionary* pDict = nullptr;

    av_dict_set(&pDict, "framerate", QString::number(m_nFps).toUtf8().data(), 0);
    av_dict_set(&pDict, "probesize", "42M", 0);
    int nRet = avformat_open_input(&m_pVideoFmtCtx, "desktop", fmt, &pDict);
    if(nRet != OK)
    {
        cout << " avformat_open_input failed:" << nRet;
        return ERR;
    }
    nRet = avformat_find_stream_info(m_pVideoFmtCtx, nullptr);
    if(nRet < OK)
    {
        cout <<  " avformat_find_stream_info failed:" << nRet;
        return ERR;
    }
    for(uint32_t i = 0; i < m_pVideoFmtCtx->nb_streams; ++i)
    {
        AVStream* pStream = m_pVideoFmtCtx->streams[i];
        if(pStream->codecpar->codec_type != AVMEDIA_TYPE_VIDEO)
            continue;
        m_nVideoIndex = i;
        AVCodec* pDecodec = avcodec_find_decoder(pStream->codecpar->codec_id);
        if(!pDecodec)
        {
            cout << "find decodec failed";
            return ERR;
        }
        m_pVideoDecodeCtx = avcodec_alloc_context3(pDecodec);
        if(!m_pVideoFmtCtx)
        {
            cout << "m_pVideoDecodeCtx alloc failed";
            return ERR;
        }
        nRet = avcodec_parameters_to_context(m_pVideoDecodeCtx, pStream->codecpar);
        if(nRet < OK)
        {
            cout << "avcodec_parameters_to_context failed:" << nRet;
            return ERR;
        }
        nRet = avcodec_open2(m_pVideoDecodeCtx, pDecodec, nullptr);
        if(nRet != OK)
        {
            cout << "open decodec failed:" << nRet;
            return ERR;
        }
        break;
    }

    m_pSwsCtx = sws_getContext(m_pVideoDecodeCtx->width, m_pVideoDecodeCtx->height, m_pVideoDecodeCtx->pix_fmt,
                   m_nWidth, m_nHeight, AV_PIX_FMT_YUV420P,
                   SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    if(!m_pSwsCtx)
    {
        cout << "sws_getContext failed";
        return ERR;
    }
    return OK;
}

int CScreenRecorder::openAudio()
{
    AVInputFormat* fmt = av_find_input_format("dshow");
    cout << m_strMicrophoneName;
    int nRet = avformat_open_input(&m_pAudioFmtCtx, m_strMicrophoneName.toUtf8().data(), fmt, nullptr);
    if(nRet != OK)
    {
        cout << "avformat_open_input failed:" << getAvErrorMsg(nRet);
        return ERR;
    }
    avformat_find_stream_info(m_pAudioFmtCtx, nullptr);
    for(uint32_t i = 0; i < m_pAudioFmtCtx->nb_streams; ++i)
    {
        AVStream* pStream = m_pAudioFmtCtx->streams[i];
        if(pStream->codecpar->codec_type != AVMEDIA_TYPE_AUDIO)
            continue;

        AVCodec* pDecodec = avcodec_find_decoder(pStream->codecpar->codec_id);
        if(!pDecodec)
        {
            cout << " find decedoc failed";
            return ERR;
        }
        m_pAudioDecodeCtx = avcodec_alloc_context3(pDecodec);
        if(!m_pAudioDecodeCtx)
        {
            cout << "m_pAudioDecodeCtx alloc failed";
            return ERR;
        }
        nRet = avcodec_parameters_to_context(m_pAudioDecodeCtx, pStream->codecpar);
        if(nRet < OK)
        {
            cout << "avcodec_parameters_to_context failed:" << nRet;
            return ERR;
        }
        nRet = avcodec_open2(m_pAudioDecodeCtx, pDecodec, nullptr);
        if(nRet != OK)
        {
            cout << "open decodec failed";
            return ERR;
        }
    }

    return OK;
}

int CScreenRecorder::openOutput()
{
    int nRet = -1;
    AVStream* pVStream = nullptr;
    AVStream* pAStream = nullptr;
    if(m_strFilePath.contains("rtmp://"))
        nRet = avformat_alloc_output_context2(&m_pOutFmtCtx, nullptr, "flv", m_strFilePath.toUtf8().data());
    else
        nRet = avformat_alloc_output_context2(&m_pOutFmtCtx, nullptr, nullptr, m_strFilePath.toUtf8().data());

    if(nRet)
    {
        cout << "avformat_alloc_output_context2 failed:" << nRet;
        return ERR;
    }

    if(m_pVideoFmtCtx->streams[m_nVideoIndex]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
    {
        pVStream = avformat_new_stream(m_pOutFmtCtx, nullptr);
        if(!pVStream)
        {
            cout << "avformat_new_stream failed";
            return ERR;
        }
        m_nVideoOutIndex = pVStream->index;
        pVStream->time_base = AVRational{1, m_nFps};
        m_pVideoEncodeCtx = avcodec_alloc_context3(nullptr);
        if(!m_pVideoEncodeCtx)
        {
            cout << "m_pVideoEncodeCtx alloc failed";
            return ERR;
        }
        m_pVideoEncodeCtx->width = m_nWidth;
        m_pVideoEncodeCtx->height = m_nHeight;
        m_pVideoEncodeCtx->codec_type = AVMEDIA_TYPE_VIDEO;

        ///// 编码器的 时间基： 帧率的倒数
        m_pVideoEncodeCtx->time_base.num = 1;        ///numerator
        m_pVideoEncodeCtx->time_base.den = m_nFps;    ///denominator

        const int nBrate = 1024 * 1024;
        m_pVideoEncodeCtx->pix_fmt = AV_PIX_FMT_YUV420P;
        m_pVideoEncodeCtx->codec_id = AV_CODEC_ID_H264;
        m_pVideoEncodeCtx->bit_rate = nBrate;
        m_pVideoEncodeCtx->rc_max_rate = nBrate;
        m_pVideoEncodeCtx->rc_buffer_size = nBrate * 4;

        m_pVideoEncodeCtx->rc_initial_buffer_occupancy = m_pVideoEncodeCtx->rc_buffer_size * 3 / 4;
        m_pVideoEncodeCtx->b_frame_strategy = 1;
        m_pVideoEncodeCtx->trellis = 2;  // 熵编码方式，2表示启用trellis优化
        m_pVideoEncodeCtx->level = 41;  // H.264编码级别
        m_pVideoEncodeCtx->refs = 1;  // 参考帧数，表示编码器可以使用多少个参考帧进行编码
        m_pVideoEncodeCtx->scenechange_threshold = 40;  // 场景切换检测的阈值，取值范围为0~100

        m_pVideoEncodeCtx->gop_size = m_nFps * 2;
        m_pVideoEncodeCtx->max_b_frames = 3;

         //设置h264中相关的参数
        m_pVideoEncodeCtx->qmin = 10;
        m_pVideoEncodeCtx->qmax = 36;
        m_pVideoEncodeCtx->max_qdiff = 4;
        m_pVideoEncodeCtx->me_range = 16;
        m_pVideoEncodeCtx->qcompress = 0.6;

        m_pVideoEncodeCtx->codec_tag = 0;
        m_pVideoEncodeCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        AVCodec* pEncodec = avcodec_find_encoder(m_pVideoEncodeCtx->codec_id);
        if(!pEncodec)
        {
            cout << "find encodec failed";
            return ERR;
        }

        nRet = avcodec_open2(m_pVideoEncodeCtx, pEncodec, nullptr);
        if(nRet < OK)
        {
            cout << "open encodec failed:" << nRet;
            return ERR;
        }
        nRet = avcodec_parameters_from_context(pVStream->codecpar, m_pVideoEncodeCtx);
        if(nRet < OK)
        {
            cout << "avcodec_parameters_from_context failed:" << nRet;
            return ERR;
        }
    }

    if(m_pAudioFmtCtx->streams[m_nAudioIndex]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
    {
        pAStream = avformat_new_stream(m_pOutFmtCtx, nullptr);
        if(!pAStream)
        {
            cout << "audio avformat_new_stream failed";
            return ERR;
        }
        m_nAudioOutIndex = pAStream->index;
        AVCodec* pEncodec = avcodec_find_encoder(AV_CODEC_ID_AAC);
        if(!pEncodec)
        {
            cout << "audio find encodec failed";
            return ERR;
        }
        m_pAudioEncodeCtx = avcodec_alloc_context3(pEncodec);
        if(!m_pAudioEncodeCtx)
        {
            cout << "m_pAudioEncodeCtx alloc failed";
            return ERR;
        }
        m_pAudioEncodeCtx->sample_fmt = pEncodec->sample_fmts ? pEncodec->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
        m_pAudioEncodeCtx->bit_rate = m_nAudioBitrate;
        m_pAudioEncodeCtx->sample_rate = 44100;
        if (pEncodec->supported_samplerates)
        {
            m_pAudioEncodeCtx->sample_rate = pEncodec->supported_samplerates[0];
            for (int i = 0; pEncodec->supported_samplerates[i]; ++i)
            {
                if (pEncodec->supported_samplerates[i] == 44100)
                    m_pAudioEncodeCtx->sample_rate = 44100;
            }
        }

        m_pAudioEncodeCtx->channel_layout = AV_CH_LAYOUT_STEREO;
        if (pEncodec->channel_layouts)
        {
            m_pAudioEncodeCtx->channel_layout = pEncodec->channel_layouts[0];
            for (int i = 0; pEncodec->channel_layouts[i]; ++i)
            {
                if (pEncodec->channel_layouts[i] == AV_CH_LAYOUT_STEREO)
                    m_pAudioEncodeCtx->channel_layout = AV_CH_LAYOUT_STEREO;
            }
        }
        m_pAudioEncodeCtx->channels = av_get_channel_layout_nb_channels(m_pAudioEncodeCtx->channel_layout);
        pAStream->time_base = AVRational{ 1, m_pAudioEncodeCtx->sample_rate };

        m_pAudioEncodeCtx->codec_tag = 0;
        m_pAudioEncodeCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

        if (!check_sample_fmt(pEncodec, m_pAudioEncodeCtx->sample_fmt))
        {
            cout << "Encoder does not support sample format " << av_get_sample_fmt_name(m_pAudioEncodeCtx->sample_fmt);
            return ERR;
        }

        nRet = avcodec_open2(m_pAudioEncodeCtx, pEncodec, nullptr);
        if(nRet < OK)
        {
            cout << "audio open encodec failed:" << nRet;
            return ERR;
        }

        nRet = avcodec_parameters_from_context(pAStream->codecpar, m_pAudioEncodeCtx);
        if(nRet < OK)
        {
            cout << "avcodec_parameters_from_context failed:" << nRet;
            return ERR;
        }


        m_pSwrCtx = swr_alloc();
        if (!m_pSwrCtx)
        {
            cout << "audio swr_alloc_set_opts failed";
            return ERR;
        }
        av_opt_set_int(m_pSwrCtx, "in_channel_count", m_pAudioDecodeCtx->channels, 0);
        av_opt_set_int(m_pSwrCtx, "in_sample_rate", m_pAudioDecodeCtx->sample_rate, 0);
        av_opt_set_sample_fmt(m_pSwrCtx, "in_sample_fmt", m_pAudioDecodeCtx->sample_fmt, 0);
        av_opt_set_int(m_pSwrCtx, "out_channel_count", m_pAudioEncodeCtx->channels, 0);
        av_opt_set_int(m_pSwrCtx, "out_sample_rate", m_pAudioEncodeCtx->sample_rate, 0);
        av_opt_set_sample_fmt(m_pSwrCtx, "out_sample_fmt", m_pAudioEncodeCtx->sample_fmt, 0);
        nRet = swr_init(m_pSwrCtx);
        if(nRet < OK)
        {
            cout << "audio swr init failed:" << nRet;
            return ERR;
        }
    }

    if(!(m_pOutFmtCtx->oformat->flags & AVFMT_NOFILE))
    {
        nRet = avio_open(&m_pOutFmtCtx->pb, m_strFilePath.toUtf8().data(), AVIO_FLAG_WRITE);
        if(nRet < OK)
        {
            cout << "avio_open failed:" <<  getAvErrorMsg(nRet);
            return ERR;
        }
    }
    nRet = avformat_write_header(m_pOutFmtCtx, nullptr);
    if(nRet < OK)
    {
        cout << "avformat_write_header failed:" <<  getAvErrorMsg(nRet);
        return ERR;
    }
    return OK;
}

AVFrame *CScreenRecorder::allocAudioFrame(AVCodecContext *_pEncodec, int _nbSamples)
{
    AVFrame *pFrame = av_frame_alloc();

    pFrame->format = _pEncodec->sample_fmt;
    pFrame->channel_layout = _pEncodec->channel_layout ? _pEncodec->channel_layout: AV_CH_LAYOUT_STEREO;
    pFrame->sample_rate = _pEncodec->sample_rate;
    pFrame->nb_samples = _nbSamples;

    if (_nbSamples)
    {
        int nRet = av_frame_get_buffer(pFrame, 0);
        if (nRet < 0)
        {
            cout << "av_frame_get_buffer failed:" << getAvErrorMsg(nRet);
            return nullptr;
        }
    }
    return pFrame;
}

void CScreenRecorder::initVideoBuffer()
{
    m_nVideoOutFrameSize = av_image_get_buffer_size(m_pVideoEncodeCtx->pix_fmt, m_nWidth, m_nHeight, 1);
    m_pVideoOutFrameBuf = (uint8_t *)av_malloc(m_nVideoOutFrameSize);
    m_pVideoOutFrame = av_frame_alloc();
    //先让AVFrame指针指向buf，后面再写入数据到buf
    av_image_fill_arrays(m_pVideoOutFrame->data, m_pVideoOutFrame->linesize, m_pVideoOutFrameBuf, m_pVideoEncodeCtx->pix_fmt, m_nWidth, m_nHeight, 1);
    //申请30帧缓存
    if (!(m_pVideoFifoBuf = av_fifo_alloc_array(30, m_nVideoOutFrameSize)))
    {
        cout << "av_fifo_alloc_array failed";
        return;
    }
}

void CScreenRecorder::initAudioBuffer()
{
    m_nbSamples = m_pAudioEncodeCtx->frame_size;
    if (!m_nbSamples)
    {
        cout << "m_nbSamples==0";
        m_nbSamples = 1024;
    }
    m_pAudioFifoBuf = av_audio_fifo_alloc(m_pAudioEncodeCtx->sample_fmt, m_pAudioEncodeCtx->channels, 30 * m_nbSamples);
    if (!m_pAudioFifoBuf)
    {
        cout << "av_audio_fifo_alloc failed";
        return;
    }
}

void CScreenRecorder::flushVideoDecoder()
{
    int nRet = -1;
    int y_size = m_nWidth * m_nHeight;
    AVFrame	*pRawFrame = av_frame_alloc();
    AVFrame *pNewFrame = av_frame_alloc();

    nRet = avcodec_send_packet(m_pVideoDecodeCtx, nullptr);
    if (nRet != 0)
    {
        cout << "flush video avcodec_send_packet failed, ret: " << nRet;
        return;
    }
    while (nRet >= 0)
    {
        nRet = avcodec_receive_frame(m_pVideoDecodeCtx, pRawFrame);
        if (nRet < 0)
        {
            if (nRet == AVERROR(EAGAIN))
            {
                cout << "flush EAGAIN avcodec_receive_frame";
                nRet = 1;
                continue;
            }
            else if (nRet == AVERROR_EOF)
            {
                cout << "flush video decoder finished";
                break;
            }
            cout << "flush video avcodec_receive_frame error, ret: " << nRet;
            return;
        }

        sws_scale(m_pSwsCtx, (const uint8_t* const*)pRawFrame->data, pRawFrame->linesize, 0, m_pVideoEncodeCtx->height, pNewFrame->data, pNewFrame->linesize);

        {
            std::unique_lock<std::mutex> lk(m_mtxVBuf);
            m_cvVBufNotFull.wait(lk, [this] { return av_fifo_space(m_pVideoFifoBuf) >= m_nVideoOutFrameSize; });
        }
        av_fifo_generic_write(m_pVideoFifoBuf, pNewFrame->data[0], y_size, nullptr);
        av_fifo_generic_write(m_pVideoFifoBuf, pNewFrame->data[1], y_size / 4, nullptr);
        av_fifo_generic_write(m_pVideoFifoBuf, pNewFrame->data[2], y_size / 4, nullptr);
        m_cvVBufNotEmpty.notify_one();
    }

}

void CScreenRecorder::flushAudioDecoder()
{
    int ret = -1;
    AVPacket pkt;
    av_init_packet(&pkt);
    int nDstNbSamples, nMaxDstNbSamples;
    AVFrame *rawFrame = av_frame_alloc();
    AVFrame *newFrame = nullptr;

    newFrame = allocAudioFrame(m_pAudioEncodeCtx, m_nbSamples);
    nMaxDstNbSamples = nDstNbSamples = av_rescale_rnd(m_nbSamples, m_pAudioEncodeCtx->sample_rate, m_pAudioDecodeCtx->sample_rate, AV_ROUND_UP);

    ret = avcodec_send_packet(m_pAudioDecodeCtx, nullptr);
    if (ret != 0)
    {
        cout << "flush audio avcodec_send_packet  failed, ret: " << ret;
        return;
    }
    while (ret >= 0)
    {
        ret = avcodec_receive_frame(m_pAudioDecodeCtx, rawFrame);
        if (ret < 0)
        {
            if (ret == AVERROR(EAGAIN))
            {
                cout << "flush audio EAGAIN avcodec_receive_frame";
                ret = 1;
                continue;
            }
            else if (ret == AVERROR_EOF)
            {
                cout << "flush audio decoder finished";
                break;
            }
            cout << "flush audio avcodec_receive_frame error, ret: " << ret;
            return;
        }

        nDstNbSamples = av_rescale_rnd(swr_get_delay(m_pSwrCtx, m_pAudioDecodeCtx->sample_rate) + rawFrame->nb_samples, m_pAudioEncodeCtx->sample_rate, m_pAudioDecodeCtx->sample_rate, AV_ROUND_UP);
        if (nDstNbSamples > nMaxDstNbSamples)
        {
            cout << "flush audio newFrame realloc";
            av_freep(&newFrame->data[0]);
            ret = av_samples_alloc(newFrame->data, newFrame->linesize, m_pAudioEncodeCtx->channels,
                nDstNbSamples, m_pAudioEncodeCtx->sample_fmt, 1);
            if (ret < 0)
            {
                cout << "flush av_samples_alloc failed";
                return;
            }
            nMaxDstNbSamples = nDstNbSamples;
            m_pAudioEncodeCtx->frame_size = nDstNbSamples;
            m_nbSamples = newFrame->nb_samples;
        }

        newFrame->nb_samples = swr_convert(m_pSwrCtx, newFrame->data, nDstNbSamples, (const uint8_t **)rawFrame->data, rawFrame->nb_samples);
        if (newFrame->nb_samples < 0)
        {
            cout << "flush swr_convert failed";
            return;
        }

        {
            std::unique_lock<std::mutex> lk(m_mtxABuf);
            m_cvABufNotFull.wait(lk, [newFrame, this] { return av_audio_fifo_space(m_pAudioFifoBuf) >= newFrame->nb_samples; });
        }
        if (av_audio_fifo_write(m_pAudioFifoBuf, (void **)newFrame->data, newFrame->nb_samples) < newFrame->nb_samples)
        {
            cout << "av_audio_fifo_write failed";
            return;
        }
        m_cvABufNotEmpty.notify_one();
    }
}

void CScreenRecorder::flushEncoders()
{
    cout << "start";
    int nRet = -1;
    bool bVideoBeginFlush = false;
    bool bAudioBeginFlush = false;

    m_nVideoCurPts = m_nAudioCurPts = 0;

    int nFlush = 2;

    while (1)
    {
        AVPacket pkt;
        av_init_packet(&pkt);

        if (av_compare_ts(m_nVideoCurPts, m_pOutFmtCtx->streams[m_nVideoOutIndex]->time_base,
            m_nAudioCurPts, m_pOutFmtCtx->streams[m_nAudioOutIndex]->time_base) <= 0)
        {

            if (!bVideoBeginFlush)
            {
                bVideoBeginFlush = true;
                nRet = avcodec_send_frame(m_pVideoEncodeCtx, nullptr);
                if (nRet != 0)
                {
                    cout << "flush video avcodec_send_frame failed, nRet: " << getAvErrorMsg(nRet);
                    return;
                }
            }
            nRet = avcodec_receive_packet(m_pVideoEncodeCtx, &pkt);
            if (nRet != OK)
            {
                av_packet_unref(&pkt);
                if (nRet == AVERROR(EAGAIN))
                {
                    cout << "flush video EAGAIN avcodec_receive_packet";
                    nRet = 1;
                    continue;
                }
                else if (nRet == AVERROR_EOF)
                {
                    cout << "flush video encoder finished";
                    if (!(--nFlush))
                        break;
                    m_nVideoCurPts = INT_MAX;
                    continue;
                }
                cout << "flush video avcodec_receive_packet failed, ret: " << nRet;
                return;
            }
            pkt.stream_index = m_nVideoOutIndex;
            //将pts从编码层的timebase转成复用层的timebase
            av_packet_rescale_ts(&pkt, m_pVideoEncodeCtx->time_base, m_pOutFmtCtx->streams[m_nVideoOutIndex]->time_base);



            m_nVideoCurPts = pkt.pts;
            cout << "m_nVideoCurPts: " << m_nVideoCurPts;

            nRet = av_interleaved_write_frame(m_pOutFmtCtx, &pkt);
            cout << "video av_interleaved_write_frame end";
            if (nRet != OK)
                cout << "flush video av_interleaved_write_frame failed, ret:" << nRet;

            av_packet_unref(&pkt);
            if (!(--nFlush))
                break;
            m_nVideoCurPts = INT_MAX;
        }
        else
        {
            if (!bAudioBeginFlush)
            {
                bAudioBeginFlush = true;
                nRet = avcodec_send_frame(m_pAudioEncodeCtx, nullptr);
                if (nRet != 0)
                {
                    cout << "flush audio avcodec_send_frame failed, ret: " << nRet;
                    return;
                }
            }
            nRet = avcodec_receive_packet(m_pAudioEncodeCtx, &pkt);
            if (nRet != OK)
            {
                av_packet_unref(&pkt);
                if (nRet == AVERROR(EAGAIN))
                {
                    cout << "flush EAGAIN avcodec_receive_packet";
                    nRet = 1;
                    continue;
                }
                else if (nRet == AVERROR_EOF)
                {
                    cout << "flush audio encoder finished";
                    /*break;*/
                    if (!(--nFlush))
                        break;
                    m_nAudioCurPts = INT_MAX;
                    continue;
                }
                cout << "flush audio avcodec_receive_packet failed, ret: " << nRet;
                return;
            }
            pkt.stream_index = m_nAudioOutIndex;
            //将pts从编码层的timebase转成复用层的timebase
            av_packet_rescale_ts(&pkt, m_pAudioEncodeCtx->time_base, m_pOutFmtCtx->streams[m_nAudioOutIndex]->time_base);

            m_nAudioCurPts = pkt.pts;
            cout << "m_nAudioCurPts: " << m_nAudioCurPts;
            nRet = av_interleaved_write_frame(m_pOutFmtCtx, &pkt);
            if (nRet != 0)
                cout << "flush audio av_interleaved_write_frame failed, ret: " << nRet;
            av_packet_unref(&pkt);
        }
    }
    cout << "end";
}

void CScreenRecorder::release()
{
    cout << "start";
    if (m_pVideoOutFrame)
    {
        av_frame_free(&m_pVideoOutFrame);
        m_pVideoOutFrame = nullptr;
    }

    if (m_pVideoOutFrameBuf)
    {
        av_free(m_pVideoOutFrameBuf);
        m_pVideoOutFrameBuf = nullptr;
    }

    if (m_pOutFmtCtx)
    {
        avio_close(m_pOutFmtCtx->pb);
        avformat_free_context(m_pOutFmtCtx);
        m_pOutFmtCtx = nullptr;
    }

    if (m_pVideoDecodeCtx)
    {
        avcodec_free_context(&m_pVideoDecodeCtx);
        m_pVideoDecodeCtx = nullptr;
    }

    if (m_pAudioDecodeCtx)
    {
        avcodec_free_context(&m_pAudioDecodeCtx);
        m_pAudioDecodeCtx = nullptr;
    }

    if (m_pVideoEncodeCtx)
    {
        avcodec_free_context(&m_pVideoEncodeCtx);
        m_pVideoEncodeCtx = nullptr;
    }

    if (m_pAudioEncodeCtx)
    {
        avcodec_free_context(&m_pAudioEncodeCtx);
        m_pAudioEncodeCtx = nullptr;
    }

    if (m_pVideoFifoBuf)
    {
        av_fifo_freep(&m_pVideoFifoBuf);
        m_pVideoFifoBuf = nullptr;
    }

    if (m_pAudioFifoBuf)
    {
        av_audio_fifo_free(m_pAudioFifoBuf);
        m_pAudioFifoBuf = nullptr;
    }

    if (m_pVideoFmtCtx)
    {
        avformat_close_input(&m_pVideoFmtCtx);
        m_pVideoFmtCtx = nullptr;
    }

    if (m_pAudioFmtCtx)
    {
        avformat_close_input(&m_pAudioFmtCtx);
        m_pAudioFmtCtx = nullptr;
    }

    if(m_pSwsCtx){
        sws_freeContext(m_pSwsCtx);
        m_pSwsCtx = nullptr;
    }

    if(m_pSwrCtx)
    {
        swr_free(&m_pSwrCtx);
        m_pSwrCtx = nullptr;
    }
    cout << "end";
}

int CScreenRecorder::check_sample_fmt(const AVCodec *codec, AVSampleFormat sample_fmt)
{
    const enum AVSampleFormat *p = codec->sample_fmts;

    while (*p != AV_SAMPLE_FMT_NONE) {
        if (*p == sample_fmt)
            return 1;
        p++;
    }
    return 0;
}

QString CScreenRecorder::getAvErrorMsg(const int &_nErr)
{
    static char buff[1024] = {0};
    av_strerror(_nErr, buff, sizeof(buff));
    return QString("ErrorCode:") + QString::number(_nErr) + QString(" Msg:")+ QString(buff);
}
















