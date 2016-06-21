#include "camhandler.h"

CamHandler::CamHandler(const int cameraId) {
    m_cameraId = cameraId;
    if(m_vc.open(m_cameraId)) {
        std::cout << "Video stream opened successfully. (id=" << m_cameraId<< ")" << std::endl;
    } else {
        std::cout << "fail opening the video capture device with id " << m_cameraId << std::endl;
    }
//    assert(m_vc.isOpened());
}

CamHandler::~CamHandler()
{
}

void CamHandler::release()
{
    m_vc.release();
    m_frame.release();
}

int CamHandler::changeResolution(double width, double height)
{
    m_vc.set(CV_CAP_PROP_FRAME_WIDTH,width);
    m_vc.set(CV_CAP_PROP_FRAME_HEIGHT,height);
}

int CamHandler::displayVideoStream()
{
    std::string windowHead = "id="+std::to_string(m_cameraId)+"- ESC to close";
    cv::namedWindow(windowHead,CV_WINDOW_AUTOSIZE);
//    cv::moveWindow(windowHead,5,5);
    while(cv::waitKey(1) != 27 && m_vc.isOpened()) {
        m_vc.read(m_frame);
        if( m_frame.empty() ) {
            std::cout << "END of video stream" << std::endl;
            return -1;
        }        
        cv::imshow(windowHead, m_frame);
    }
    return 0;
}

cv::VideoCapture CamHandler::getVideoCapture() {
    return m_vc;
}

int CamHandler::getCameraId() {
    return m_cameraId;
}

cv::Mat CamHandler::getFrame()
{
    return m_frame;
}

double CamHandler::getParameter(std::string param)
{
    if (param.compare("posInVideo")==0) {
        return m_vc.get(CV_CAP_PROP_POS_MSEC);
    }
    if(param.compare("indexOfFram")==0) {
        return m_vc.get(CV_CAP_PROP_POS_FRAMES);
    }
    if(param.compare("posInVideoFile")==0) {
        return m_vc.get(CV_CAP_PROP_POS_AVI_RATIO);
    }
    if(param.compare("width") == 0) {
        return m_vc.get(CV_CAP_PROP_FRAME_WIDTH);
    }
    if(param.compare("height") == 0) {
        return m_vc.get(CV_CAP_PROP_FRAME_HEIGHT);
    }
    if(param.compare("fps")==0) {
        return m_vc.get(CV_CAP_PROP_FPS);
    }
    if(param.compare("fourcc")==0) {
        return m_vc.get(CV_CAP_PROP_FOURCC);
    }
    if(param.compare("frameNb")==0) {
        return m_vc.get(CV_CAP_PROP_FRAME_COUNT);
    }
    if(param.compare("format")==0) {
        return m_vc.get(CV_CAP_PROP_FORMAT);
    }
    if(param.compare("captureMode")==0) {
        return m_vc.get(CV_CAP_PROP_MODE);
    }
    if(param.compare("brightness")==0) {
        return m_vc.get(CV_CAP_PROP_BRIGHTNESS);
    }
    if(param.compare("contrast")==0) {
        return m_vc.get(CV_CAP_PROP_CONTRAST);
    }
    if(param.compare("saturation")==0) {
        return m_vc.get(CV_CAP_PROP_SATURATION);
    }
    if(param.compare("hue")==0) {
        return m_vc.get(CV_CAP_PROP_HUE);
    }
    if(param.compare("gain")==0) {
        return m_vc.get(CV_CAP_PROP_GAIN);
    }
    if(param.compare("exposure")==0) {
        return m_vc.get(CV_CAP_PROP_EXPOSURE);
    }
    if(param.compare("convert2RGB")==0) {
        return m_vc.get(CV_CAP_PROP_CONVERT_RGB);
    }
    if(param.compare("rectification")==0) {
        return m_vc.get(CV_CAP_PROP_RECTIFICATION);
    }
    return -1;
}

bool CamHandler::setParameter(std::string param, double value)
{
    if(param.compare("width") == 0) {
        return m_vc.set(CV_CAP_PROP_FRAME_WIDTH,value);
    }
    if(param.compare("height") == 0) {
        return m_vc.set(CV_CAP_PROP_FRAME_HEIGHT,value);
    }
    if (param.compare("posInVideo")==0) {
        return m_vc.set(CV_CAP_PROP_POS_MSEC,value);
    }
    if(param.compare("indexOfFram")==0) {
        return m_vc.set(CV_CAP_PROP_POS_FRAMES,value);
    }
    if(param.compare("posInVideoFile")==0) {
        return m_vc.set(CV_CAP_PROP_POS_AVI_RATIO,value);
    }
    if(param.compare("fps")==0) {
        return m_vc.set(CV_CAP_PROP_FPS,value);
    }
    if(param.compare("fourcc")==0) {
        return m_vc.set(CV_CAP_PROP_FOURCC,value);
    }
    if(param.compare("frameNb")==0) {
        return m_vc.set(CV_CAP_PROP_FRAME_COUNT,value);
    }
    if(param.compare("format")==0) {
        return m_vc.set(CV_CAP_PROP_FORMAT,value);
    }
    if(param.compare("captureMode")==0) {
        return m_vc.set(CV_CAP_PROP_MODE,value);
    }
    if(param.compare("brightness")==0) {
        return m_vc.set(CV_CAP_PROP_BRIGHTNESS,value);
    }
    if(param.compare("contrast")==0) {
        return m_vc.set(CV_CAP_PROP_CONTRAST,value);
    }
    if(param.compare("saturation")==0) {
        return m_vc.set(CV_CAP_PROP_SATURATION,value);
    }
    if(param.compare("hue")==0) {
        return m_vc.set(CV_CAP_PROP_HUE,value);
    }
    if(param.compare("gain")==0) {
        return m_vc.set(CV_CAP_PROP_GAIN,value);
    }
    if(param.compare("exposure")==0) {
        return m_vc.set(CV_CAP_PROP_EXPOSURE,value);
    }
    if(param.compare("convert2RGB")==0) {
        return m_vc.set(CV_CAP_PROP_CONVERT_RGB,value);
    }
    if(param.compare("rectification")==0) {
        return m_vc.set(CV_CAP_PROP_RECTIFICATION,value);
    }
    return false;
}

bool CamHandler::grab()
{
    bool output = m_vc.grab();
    if(!output) {
        std::cout << "Failed grabbing a frame" << std::endl;
    }
    return output;
}

bool CamHandler::retrieve()
{
    bool isGrabbed = m_vc.retrieve(m_frame);
    if(!isGrabbed) {
        std::cout << "fail grabbing the image" << std::endl;
        m_frame = cv::Scalar(255,255,255);
    }
    return isGrabbed;
}

bool CamHandler::isOpened()
{
    return m_vc.isOpened();
}

cv::Mat CamHandler::read() {
	cv::Mat frame;
	bool retCode = m_vc.read(frame);
	m_frame = frame;
	return frame;
}
