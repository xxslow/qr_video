/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 2.1 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**********/
// Copyright (c) 1996-2016, Live Networks, Inc.  All rights reserved
// A demo application, showing how to create and run a RTSP client (that can potentially receive multiple streams concurrently).
//
// NOTE: This code - although it builds a running application - is intended only to illustrate how to develop your own RTSP
// client applicdefineation.  For a full-featured RTSP client application - with much more functionality, and many options - see
// "openRTSP": http://www.live555.com/openRTSP/
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <net/if.h>
#include <stdlib.h>
#include <netinet/icmp6.h>
#include <sys/time.h>
#include <pthread.h>
#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

#include "v2x_general.h"
#include "v2x_wireless_types.h"

#include <fcntl.h>

//�ڲ�ͷ�ļ�
#include "peer_video.h"

//#define DEBUG_COMMAND
//#define DEBUG_NUM


char eventLoopWatchVariable;	//0:�߳�������;1:�ر��߳�;2:�̹߳ر���
static unsigned int s_f_cnt = 0;	//��Ƶ֡����
int rstp_stream_runing_flag = 0;
int recv_stream_flag = 0;
int packet_count = 0;
int r_packet_count = 0;
int msg_type_count = 1;

//====================================================================
//��̬����
static struct video_thr s_video_thr;	//�����߳���Ϣ
static struct com_sock_struct s_com_socks[MAX_SOCK_NUM];
static struct clt_sock_struct s_wmh_sock;
static struct clt_sock_struct s_dvin_sock;
static struct com_sock_struct *s_com_wmh_sock;
static struct com_sock_struct *s_com_dvin_sock;
//xzq
static RTSPClient* s_my_rtspClient = NULL;
FILE *pFile;

//====================================================================
/*************************************************
 ��������:    close_socks
 ��������:    �ر�socks
 ���ߣ�
 �������?    ��
 �������?    ��
 ����˵��:    ��
 ����˵��:    ��
 *************************************************/
static void close_socks()
{
    int i;
    for(i = 0; i < MAX_SOCK_NUM; i++)
    {
        if(s_com_socks[i].fd > 0)
        {
            close(s_com_socks[i].fd);
        }
    }
}
// Forward function definitions:

// RTSP 'response handlers':
void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString);
void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString);

// Other event handler functions:
void subsessionAfterPlaying(void* clientData); // called when a stream's subsession (e.g., audio or video substream) ends
void subsessionByeHandler(void* clientData); // called when a RTCP "BYE" is received for a subsession
void streamTimerHandler(void* clientData);
  // called at the end of a stream's expected duration (if the stream has not already signaled its end using a RTCP "BYE")

// The main streaming routine (for each "rtsp://" URL):
void openURL(UsageEnvironment& env, char const* progName, char const* rtspURL);

// Used to iterate through each stream's 'subsessions', setting up each one:
void setupNextSubsession(RTSPClient* rtspClient);

// Used to shut down and close a stream (including its "RTSPClient" object):
void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

// A function that outputs a string that identifies each stream (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env, const RTSPClient& rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env, const MediaSubsession& subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

void usage(UsageEnvironment& env, char const* progName) {
  env << "Usage: " << progName << " <rtsp-url-1> ... <rtsp-url-N>\n";
  env << "\t(where each <rtsp-url-i> is a \"rtsp://\" URL)\n";
}

void deal_signal(int n)
{
	printf("signal trigered\n");
	if (s_my_rtspClient != NULL){
		shutdownStream(s_my_rtspClient);
		s_my_rtspClient = NULL;
	}
	fclose(pFile); 
	fflush(pFile);

	close_socks();
	exit(0);
}


/*************************************************
 ��������:    deal_decode_info
 ��������:    ���������Ϣ�����͵�dvin��
 ���ߣ�
 �������?    buf���������ݻ��棻type��������������
 �������?    ��
 ����˵��:    ��
 ����˵��:    ��
 *************************************************/
static int deal_decode_info(struct com_sock_struct* com_sock, char *buf, int type)
{
    int ret = -1;
    char tx_buf[MAX_BUF_LEN] = {0};
	char tmp_vis_udm_buf[MAX_BUF_LEN] = {0};
	struct wmh_udm_struct *tmp_wmh_udm;
	struct vis_udm_struct *tx_vis_udm;
	
    switch(type)
    {
        case INFO_WMH_UDM:
        {
			tmp_wmh_udm = (struct wmh_udm_struct *)buf;
			//if (tmp_wmh_udm->msg_type == 1){
				tx_vis_udm = (struct vis_udm_struct *)tmp_vis_udm_buf;
				tx_vis_udm->vi_type = 0x02;
				tx_vis_udm->msg_type = 1;
				tx_vis_udm->length = tmp_wmh_udm->length;
				memcpy(tx_vis_udm->data, tmp_wmh_udm->data, tx_vis_udm->length);
				
				//printf("r_len %d ",tmp_wmh_udm->length);
				//printf("r_packet_count %d \n",++r_packet_count);
				
				ret = encode_internal_message(INFO_VIS_UDM, tmp_vis_udm_buf, tx_buf);
				if(ret){
					v2x_pr(LOG_LEVEL_WARNING, UDM_V_LOG_ID, "encode_internal_message INFO_VIS_UDM err");
					return -1;
				}
				sendto(s_com_dvin_sock->fd, tx_buf, (strlen(tx_buf) + 1), 0, (struct sockaddr *)&(s_com_dvin_sock->sockaddr), sizeof(s_com_dvin_sock->sockaddr));
				//printf("recv:%d type:%d\n",++r_packet_count, tmp_wmh_udm->msg_type);
			//}

            break;
        }
        default:
        {
            break;
        }
    }
    return 0;
}
/*************************************************
 ��������:    on_recv
 ��������:    ���տͻ�����Ϣ
 ���ߣ�
 �������?    com_sock��ͨ��socket�� buf���������ݻ��棻len���������ݳ��ȣ�
 �������?    ��
 ����˵��:    0���ɹ�����0��ʧ��
 ����˵��:    ��
 *************************************************/
static int on_recv(struct com_sock_struct* com_sock, char* buf, int len)
{
    return com_recv_deal(com_sock, LOG_ID, buf, len, deal_decode_info);
}

//====================================================================
//������   : init_app
//�������� : �豸��ʼ��
//����     : ��
//����ֵ   : ��
//====================================================================
void init_app(void)
{
	memset(&s_video_thr, 0, sizeof(struct video_thr));
	s_video_thr.tid_close_flg_1 = 2;
	eventLoopWatchVariable = 2;
	printf("init app done\n");
}

/*************************************************
 ��������:    init_socks
 ��������:    ��ʼ��socks
 ���ߣ�
 �������?    ��
 �������?    ��
 ����˵��:    ��
 ����˵��:    ��
 *************************************************/
static void init_socks()
{
    //��ʼ������SOCKS
    int i;
    for(i = 0; i < MAX_SOCK_NUM; i++)
    {
        general_strcut_init((void *)&s_com_socks[i], INFO_COMM_SOCK);
    }
    //��ʼ��WMH SOCKET
    general_strcut_init((void *)&s_wmh_sock,INFO_CLIENT_SOCK);
    s_wmh_sock.on_recv = on_recv;
    s_wmh_sock.reg_type = INFO_WMH_RM;
    struct wmh_rm_struct* wmh_rm_info = (struct wmh_rm_struct*)s_wmh_sock.reg_info;
    wmh_rm_info->msg_type = MSG_TYPE_WMH_UDM;
    
    s_com_wmh_sock = &s_com_socks[0];
    s_com_wmh_sock->ex = (void *)&s_wmh_sock;
    s_com_wmh_sock->type = ST_TCP_CLIENT;
    sprintf(s_com_wmh_sock->name, "wmh");
    sprintf(s_com_wmh_sock->addr, LOCAL_INET_ADDR);
    s_com_wmh_sock->port = WMH_TCP_PORT;
	
	//��ʼ��DVIN SOCKET
	general_strcut_init((void *)&s_dvin_sock,INFO_CLIENT_SOCK);
    //s_dvin_sock.on_recv = on_recv;
	s_com_dvin_sock = &s_com_socks[1];
    s_com_dvin_sock->ex = (void *)&s_dvin_sock;
    s_com_dvin_sock->type = ST_UDP_CLIENT;
    sprintf(s_com_dvin_sock->name,"dvin");
    sprintf(s_com_dvin_sock->addr,LOCAL_INET_ADDR);
    s_com_dvin_sock->port = DVIN_UDP_PORT;
}

//====================================================================
//������   : thr_send_video_data
//�������� : ����������Ƶ����
//����     : ��
//����ֵ   : ��
//====================================================================
void *thr_send_video_data(void*)
{
	printf("thr_send_video_data started\n");
	while (1){
		if (rstp_stream_runing_flag == 0){
			rstp_stream_runing_flag = 1; // either delete or not
			TaskScheduler* scheduler = BasicTaskScheduler::createNew();
			UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
			eventLoopWatchVariable = 0;
			s_f_cnt = 0;
			openURL(*env, "peer_video", 
			"rtsp://admin:abcd1234@192.169.5.210/h264/ch1/main/av_stream");
			printf("open url done\n");
			env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
			printf("loop done, start reclaim\n");
			if (s_my_rtspClient != NULL){
				shutdownStream(s_my_rtspClient);
				s_my_rtspClient = NULL;
			}
			env->reclaim();
			env = NULL;
			delete scheduler; 
			scheduler = NULL;
			eventLoopWatchVariable = 2;
		}
		rstp_stream_runing_flag = 0;
		sleep(3); // wait 3s for recover and re-connection
	}

	return ((void *)0);
}

//====================================================================
//������   : main
//�������� : ������
//����     : argc������������argv������ָ��
//����ֵ   : ��
//====================================================================
int main(int argc, char** argv)
{
	//����SIGPIPE�źţ���ֹ���ӶϿ�ʱ����SIGPIPE�ź���ֹ����
    signal(SIGPIPE, SIG_IGN);
	
	signal(SIGINT, deal_signal); 
	char read_buf[1024];
	char num_buf[10] = {0};
	char param;
	int i = 0;
	int pos = 0;
	char vis_tx_buf[MAX_BUF_LEN] = {0};
	char vis_tmp_buf[MAX_BUF_LEN] = {0};
	struct vis_udm_struct *tx_vis_udm;
	
	if (argc != 2){
		printf("add parameters:\n");
		printf("v2x_appv_txv 0 for broadcast only\n");
		printf("v2x_appv_txv 1 for receive only\n");
		printf("v2x_appv_txv 2 for both broadcast and receive\n");
		goto End;
	}
	else{
		param = argv[1][0];
	}
		
    //��ʼ��
	init_app();
    init_socks();
    if(com_create_socks(s_com_socks, MAX_SOCK_NUM, LOG_ID))
    {
        v2x_pr(LOG_LEVEL_ERROR, LOG_ID, "com_create_socks err");
        return -1;
    }
	
	pFile = fopen("capture_video", "r");
	
	tx_vis_udm = (struct vis_udm_struct *)(vis_tmp_buf);
	tx_vis_udm->vi_type = 0x02;
	tx_vis_udm->msg_type = 1;
	
	while(1)
	{
		if (feof(pFile) != 0)
			break;
		fread(read_buf,1,1,pFile);
		if (read_buf[0] != ' '){
			num_buf[i] = read_buf[0];
			i++;
			pos++;
			continue;
		}
		num_buf[i] = '\0';
		i = 0;
		int cnt = atoi(num_buf);
		printf("cnt %d\n", cnt);
		fread(read_buf,1,cnt,pFile);
		tx_vis_udm->length = cnt;
		memcpy(tx_vis_udm->data, read_buf, tx_vis_udm->length);
		
		sendto(s_com_dvin_sock->fd, vis_tx_buf, (strlen(vis_tx_buf) + 1), 0, (struct sockaddr *)&(s_com_dvin_sock->sockaddr), sizeof(s_com_dvin_sock->sockaddr));
		usleep(1000);
	}

	/*
	// start send video frame thread first
	if((param == '0' || param == '2') && eventLoopWatchVariable){			
		if(pthread_create(&s_video_thr.tid_0, NULL, thr_send_video_data, NULL) != 0) {
			printf("thr_send_video_data start failed\n");
			eventLoopWatchVariable = 2;
			goto End;
		}
		if(pthread_detach(s_video_thr.tid_0) != 0) {
			printf("detach thr_send_video_data failed\n");
			goto End;
		}
	}
	
    while(1)
    {
		//select ʱ��
		struct timeval rx_timeout;
		rx_timeout.tv_sec = 5;
		rx_timeout.tv_usec = 0;
        com_select_socks(s_com_socks, MAX_SOCK_NUM, LOG_ID, &rx_timeout);
		com_create_socks(s_com_socks, MAX_SOCK_NUM, LOG_ID);
    }
	*/
    close_socks();
    return 0;

End:
	printf("peer_video end\n");
	close_socks();
	return 0;
}


// Define a class to hold per-stream state that we maintain throughout each stream's lifetime:

class StreamClientState {
public:
  StreamClientState();
  virtual ~StreamClientState();

public:
  MediaSubsessionIterator* iter;
  MediaSession* session;
  MediaSubsession* subsession;
  TaskToken streamTimerTask;
  double duration;
};

// If you're streaming just a single stream (i.e., just from a single URL, once), then you can define and use just a single
// "StreamClientState" structure, as a global variable in your application.  However, because - in this demo application - we're
// showing how to play multiple streams, concurrently, we can't do that.  Instead, we have to have a separate "StreamClientState"
// structure for each "RTSPClient".  To do this, we subclass "RTSPClient", and add a "StreamClientState" field to the subclass:

class ourRTSPClient: public RTSPClient {
public:
  static ourRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL,
				  int verbosityLevel = 0,
				  char const* applicationName = NULL,
				  portNumBits tunnelOverHTTPPortNum = 0);

protected:
  ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
		int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum);
    // called only by createNew();
  virtual ~ourRTSPClient();

public:
  StreamClientState scs;
};

// Define a data sink (a subclass of "MediaSink") to receive the data for each subsession (i.e., each audio or video 'substream').
// In practice, this might be a class (or a chain of classes) that decodes and then renders the incoming audio or video.
// Or it might be a "FileSink", for outputting the received data into a file (as is done by the "openRTSP" application).
// In this example code, however, we define a simple 'dummy' sink that receives incoming data, but does nothing with it.

class DummySink: public MediaSink {
public:
  static DummySink* createNew(UsageEnvironment& env,
			      MediaSubsession& subsession, // identifies the kind of data that's being received
			      char const* streamId = NULL); // identifies the stream itself (optional)

private:
  DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId);
    // called only by "createNew()"
  virtual ~DummySink();

  static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
				struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
			 struct timeval presentationTime, unsigned durationInMicroseconds);
			 
  void snd_frame_to_wmh(unsigned f_len);

private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();

private:
  u_int8_t* f_all_buf;
  u_int8_t* fReceiveBuffer;
  MediaSubsession& fSubsession;
  char* fStreamId;
};

#define RTSP_CLIENT_VERBOSITY_LEVEL 1 // by default, print verbose output from each "RTSPClient"

static unsigned rtspClientCount = 0; // Counts how many streams (i.e., "RTSPClient"s) are currently in use.

void openURL(UsageEnvironment& env, char const* progName, char const* rtspURL) {
  // Begin by creating a "RTSPClient" object.  Note that there is a separate "RTSPClient" object for each stream that we wish
  // to receive (even if more than stream uses the same "rtsp://" URL).
  RTSPClient* rtspClient = ourRTSPClient::createNew(env, rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
  if (rtspClient == NULL) {
    env << "Failed to create a RTSP client for URL \"" << rtspURL << "\": " << env.getResultMsg() << "\n";
    return;
  }

  ++rtspClientCount;

  // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the stream.
  // Note that this command - like all RTSP commands - is sent asynchronously; we do not block, waiting for a response.
  // Instead, the following function call returns immediately, and we handle the RTSP response later, from within the event loop:
  rtspClient->sendDescribeCommand(continueAfterDESCRIBE); 
}


// Implementation of the RTSP 'response handlers':

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString << "\n";
      delete[] resultString;
      break;
    }

    char* const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription; // because we don't need it anymore
    if (scs.session == NULL) {
      env << *rtspClient << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
      break;
    }

    // Then, create and set up our data source objects for the session.  We do this by iterating over the session's 'subsessions',
    // calling "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command, on each one.
    // (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    setupNextSubsession(rtspClient);
    return;
  } while (0);

  // An unrecoverable pr_error occurred with this stream.
  shutdownStream(rtspClient);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP, change the following to True:
#define REQUEST_STREAMING_OVER_TCP False

void setupNextSubsession(RTSPClient* rtspClient) {
  UsageEnvironment& env = rtspClient->envir(); // alias
  StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias
  
  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL) {
    if (!scs.subsession->initiate()) {
      env << *rtspClient << "Failed to initiate the \"" << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
      setupNextSubsession(rtspClient); // give up on this subsession; go to the next one
    } else {
      env << *rtspClient << "Initiated the \"" << *scs.subsession << "\" subsession (";
      if (scs.subsession->rtcpIsMuxed()) {
	env << "client port " << scs.subsession->clientPortNum();
      } else {
	env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum()+1;
      }
      env << ")\n";

      // Continue setting up this subsession, by sending a RTSP "SETUP" command:
      rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, REQUEST_STREAMING_OVER_TCP);
    }
    return;
  }

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY" command to start the streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute' time, so send an appropriate "PLAY" command:
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession << "\" subsession: " << resultString << "\n";
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession << "\" subsession (";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "client port " << scs.subsession->clientPortNum();
    } else {
      env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum()+1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and call "startPlaying()" on it.
    // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
    // after we've sent a RTSP "PLAY" command.)

	//xzq
    scs.subsession->sink = DummySink::createNew(env, *scs.subsession, rtspClient->url());
      // perhaps use your own custom "MediaSink" subclass instead
    if (scs.subsession->sink == NULL) {
      env << *rtspClient << "Failed to create a data sink for the \"" << *scs.subsession
	  << "\" subsession: " << env.getResultMsg() << "\n";
      break;
    }

    env << *rtspClient << "Created a data sink for the \"" << *scs.subsession << "\" subsession\n";
    scs.subsession->miscPtr = rtspClient; // a hack to let subsession handler functions get the "RTSPClient" from the subsession 
    //xzq a hack to shutdown strean when command 7 is received.
	s_my_rtspClient = rtspClient;
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
				       subsessionAfterPlaying, scs.subsession);
    // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
    }
  } while (0);
  delete[] resultString;

  // Set up the next subsession, if any:
  setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
  Boolean success = False;

  do {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias
    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString << "\n";
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration (if the stream does not already signal its end
    // using a RTCP "BYE").  This is optional.  If, instead, you want to keep the stream active - e.g., so you can later
    // 'seek' back within it and do another RTSP "PLAY" - then you can omit this code.
    // (Alternatively, if you don't want to receive the entire stream, you could set this timer for some shorter value.)
    if (scs.duration > 0) {
      unsigned const delaySlop = 1; // number of seconds extra to delay, after the stream's expected duration.  (This is optional.)
      scs.duration += delaySlop;
      unsigned uSecsToDelay = (unsigned)(scs.duration*1000000);
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*)streamTimerHandler, rtspClient);
    }

    env << *rtspClient << "Started playing session";
    if (scs.duration > 0) {
      env << " (for up to " << scs.duration << " seconds)";
    }
    env << "...\n";

    success = True;
  } while (0);
  delete[] resultString;

  if (!success) {
    // An unrecoverable pr_error occurred with this stream.
    shutdownStream(rtspClient);
  }
}


// Implementation of the other event handlers:

void subsessionAfterPlaying(void* clientData) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession& session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    if (subsession->sink != NULL) return; // this subsession is still active
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)subsession->miscPtr;
  UsageEnvironment& env = rtspClient->envir(); // alias

  env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession << "\" subsession\n";

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void* clientData) {
  ourRTSPClient* rtspClient = (ourRTSPClient*)clientData;
  StreamClientState& scs = rtspClient->scs; // alias

  scs.streamTimerTask = NULL;

  // Shut down the stream:
  shutdownStream(rtspClient);
}

void shutdownStream(RTSPClient* rtspClient, int exitCode) {
  UsageEnvironment& env = rtspClient->envir(); // alias
  StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL) { 
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession* subsession;

    while ((subsession = iter.next()) != NULL) {
      if (subsession->sink != NULL) {
	Medium::close(subsession->sink);
	subsession->sink = NULL;

	if (subsession->rtcpInstance() != NULL) {
	  subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
	}

	someSubsessionsWereActive = True;
      }
    }

    if (someSubsessionsWereActive) {
      // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
      // Don't bother handling the response to the "TEARDOWN".
      rtspClient->sendTeardownCommand(*scs.session, NULL);
	  //xzq
	  printf("RTSP teardown has send to server\n");
    }
  }
  
  env << *rtspClient << "Closing the stream.\n";
  Medium::close(rtspClient);
    // Note that this will also cause this stream's "StreamClientState" structure to get reclaimed.

  if (--rtspClientCount == 0) {
    // The final stream has ended, so exit the application now.
    // (Of course, if you're embedding this code into your own application, you might want to comment this out,
    // and replace it with "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop, and continue running "main()".)
    //exit(exitCode);
	eventLoopWatchVariable = 1;
  }
}


// Implementation of "ourRTSPClient":

ourRTSPClient* ourRTSPClient::createNew(UsageEnvironment& env, char const* rtspURL,
					int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum) {
  return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
			     int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum)
  : RTSPClient(env,rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum, -1) {
}

ourRTSPClient::~ourRTSPClient() {
}


// Implementation of "StreamClientState":

StreamClientState::StreamClientState()
  : iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL), duration(0.0) {
}

StreamClientState::~StreamClientState() {
  delete iter;
  if (session != NULL) {
    // We also need to delete "session", and unschedule "streamTimerTask" (if set)
    UsageEnvironment& env = session->envir(); // alias

    env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
    Medium::close(session);
  }
}


// Implementation of "DummySink":

// Even though we're not going to be doing anything with the incoming data, we still need to receive it.
// Define the size of the buffer that we'll use:
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 80000

DummySink* DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId) {
  return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId)
  : MediaSink(env),
    fSubsession(subsession) {
  fStreamId = strDup(streamId);
  f_all_buf = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE + 4];
  f_all_buf[0] = 0x00;
  f_all_buf[1] = 0x00;
  f_all_buf[2] = 0x00;
  f_all_buf[3] = 0x01;
  fReceiveBuffer = f_all_buf + 4;
}

DummySink::~DummySink() {
  //delete[] fReceiveBuffer;
  delete[] f_all_buf;
  delete[] fStreamId;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes,
				  struct timeval presentationTime, unsigned durationInMicroseconds) {
  DummySink* sink = (DummySink*)clientData;
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then comment out the following line:
//#define DEBUG_PRINT_EACH_RECEIVED_FRAME 1

void DummySink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
				  struct timeval presentationTime, unsigned /*durationInMicroseconds*/) {
  // We've just received a frame of data.  (Optionally) print out information about it:
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (fStreamId != NULL) envir() << "Stream \"" << fStreamId << "\"; ";
  envir() << fSubsession.mediumName() << "/" << fSubsession.codecName() << ":\tReceived " << frameSize << " bytes";
  if (numTruncatedBytes > 0) envir() << " (with " << numTruncatedBytes << " bytes truncated)";
  char uSecsStr[6+1]; // used to output the 'microseconds' part of the presentation time
  sprintf(uSecsStr, "%06u", (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: " << (int)presentationTime.tv_sec << "." << uSecsStr;
  if (fSubsession.rtpSource() != NULL && !fSubsession.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!"; // mark the debugging output to indicate that this presentation time is not RTCP-synchronized
  }
#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << fSubsession.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif

  snd_frame_to_wmh(frameSize + 4);
  // Then continue, to request the next frame of data:
  continuePlaying();
}

Boolean DummySink::continuePlaying() {
  if (fSource == NULL) return False; // sanity check (should not happen)

  // Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
  fSource->getNextFrame(fReceiveBuffer, DUMMY_SINK_RECEIVE_BUFFER_SIZE,
                        afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}
//====================================================================
//������   : snd_frame_to_wmh
//�������� : ���ͻ�ȡ��������wmhģ��
//����     : ��
//����ֵ   : ��
//====================================================================
void DummySink::snd_frame_to_wmh(unsigned f_len)
{
	int cnt, sum, left_len, head_len, i;
	char frame_len[10] = {0};
	
	// construct udm wireless type
	char tx_buf[MAX_BUF_LEN] = {0};
	char tmp_buf[MAX_BUF_LEN] = {0};
	char* snd_buf;
	struct wmh_udm_struct *wmh_udm;
	int encode_ret;
	
	//debug
	char vis_tx_buf[MAX_BUF_LEN] = {0};
	char vis_tmp_buf[MAX_BUF_LEN] = {0};
	struct vis_udm_struct *tx_vis_udm;
	
	// to hold current whole frame
	char* tmp_all_buf = new char[f_len];
	memcpy(tmp_all_buf, f_all_buf, f_len);
	
	// prevent disconnect
	if(s_com_wmh_sock->fd <= 0){
        goto SEND_END;
	}
	
    wmh_udm = (struct wmh_udm_struct *)(tmp_buf);
    general_strcut_init((void *)wmh_udm, INFO_WMH_UDM);
	
	//debug
	tx_vis_udm = (struct vis_udm_struct *)(vis_tmp_buf);
	tx_vis_udm->vi_type = 0x02;
	tx_vis_udm->msg_type = 1;
    				
	snd_buf = wmh_udm->data;
	s_f_cnt++;
	cnt = 0;
	sum = f_len / LEN_MAX_PER_FRAME + 1;
	left_len = f_len - LEN_MAX_PER_FRAME * (sum - 1);
	for(i = 0; i < (sum - 1); i++, cnt++)
	{
		wmh_udm->msg_type = msg_type_count;
		sprintf(snd_buf, "V %u %u %u ", s_f_cnt, sum, (cnt + 1));
		head_len = strlen(snd_buf);
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
	envir() << snd_buf << "send " << head_len + LEN_MAX_PER_FRAME << " byte\n";
#endif

#ifdef DEBUG_NUM
	printf("sendtoWMH: %u %u %u\n", s_f_cnt, sum, cnt + 1);
#endif
		if(tmp_all_buf == NULL)
			goto SEND_END;
		memcpy((snd_buf + head_len), (tmp_all_buf + (cnt * LEN_MAX_PER_FRAME)), LEN_MAX_PER_FRAME);

		//debug copy before encode
		tx_vis_udm->length = head_len + LEN_MAX_PER_FRAME;
		memcpy(tx_vis_udm->data, snd_buf, tx_vis_udm->length);
		
		sprintf(frame_len, "%d ", head_len + LEN_MAX_PER_FRAME);
		fwrite (frame_len,
				1,
				strlen(frame_len),
				pFile);
		fwrite (snd_buf,
				1,
				head_len + LEN_MAX_PER_FRAME,
				pFile);
		
		wmh_udm->length = head_len + LEN_MAX_PER_FRAME;
		//printf("len %d ",wmh_udm->length);
		//encode_internal_message(INFO_WMH_UDM, tmp_buf, tx_buf);
		//send(s_com_wmh_sock->fd, tx_buf, (strlen(tx_buf) + 1), 0);
		
		//debug
		//encode_internal_message(INFO_VIS_UDM, vis_tmp_buf, vis_tx_buf);
		//sendto(s_com_dvin_sock->fd, vis_tx_buf, (strlen(vis_tx_buf) + 1), 0, (struct sockaddr *)&(s_com_dvin_sock->sockaddr), sizeof(s_com_dvin_sock->sockaddr));
		
		//usleep(200);
		//msg_type_count++;
		//printf("packet_count %d\n",++packet_count);

#ifdef DEBUG_NUM
	printf("sendtocd: %0.23s\n", buf); // print buf[0] to buf[23]
#endif
	}
	
	//send left data
	memset(wmh_udm->data, 0, MAX_PT_BUF_LEN);
	sprintf(snd_buf, "V %u %u %u ", s_f_cnt, sum, sum);
	head_len = strlen(snd_buf);
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
	envir() << snd_buf << "send " << head_len + left_len << " byte\n";
#endif

#ifdef DEBUG_NUM
	printf("sendtoOBU: %u %u %u\n", s_f_cnt, sum, sum);
#endif
	if(tmp_all_buf == NULL)
		goto SEND_END;
	memcpy((snd_buf + head_len), (tmp_all_buf + (cnt * LEN_MAX_PER_FRAME)), left_len);
	wmh_udm->msg_type = msg_type_count;
	wmh_udm->length = head_len + left_len;
	
	//debug copy before encode
	tx_vis_udm->length = head_len + left_len;
	memcpy(tx_vis_udm->data, snd_buf, tx_vis_udm->length);
	
	sprintf(frame_len, "%d ", head_len + left_len);
	fwrite (frame_len,
				1,
				strlen(frame_len),
				pFile);
	
	fwrite (snd_buf,
			1,
			head_len + left_len,
			pFile);
	//encode_internal_message(INFO_WMH_UDM, tmp_buf, tx_buf);
    //send(s_com_wmh_sock->fd, tx_buf, (strlen(tx_buf) + 1), 0);
	
	//debug
	//encode_internal_message(INFO_VIS_UDM, vis_tmp_buf, vis_tx_buf);
	//sendto(s_com_dvin_sock->fd, vis_tx_buf, (strlen(vis_tx_buf) + 1), 0, (struct sockaddr *)&(s_com_dvin_sock->sockaddr), sizeof(s_com_dvin_sock->sockaddr));
	
	//usleep(200);
	//msg_type_count++;
	//printf("packet_count %d\n",++packet_count);
	
	
SEND_END:
	if (tmp_all_buf != NULL)
		delete[] tmp_all_buf;
}
