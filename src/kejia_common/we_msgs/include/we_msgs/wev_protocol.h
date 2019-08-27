#ifndef __WEV_PROTOCOL
#define __WEV_PROTOCOL

namespace we_vision
{
// "vp" prefix stands for Vision Prtocol

/// @todo make these extern?
// Vision command string
// Run commands
static const char* VP_RUN_IDLE          = "IDLE";
static const char* VP_RESET_ALL         = "RESET";
static const char* VP_RUN_FM_ENROLL     = ":TrackPeople 1";
static const char* VP_RUN_FM_WAVE       = ":WaveDetector 1";
static const char* VP_RUN_FACE_ENROLL   = ":RecognizeFace 1";
static const char* VP_RUN_FACE_RECOG    = ":RecognizeFace 2";
static const char* VP_RUN_FACE_DET      = ":DetectPeopleByFace 2";
static const char* VP_RUN_FACE_DET_ODOM = ":DetectPeopleByFace 1";
static const char* VP_RUN_WAVE_DET      = ":WaveDetector 2";
static const char* VP_RUN_FLR_PEO_DET   = "FLR_PEO_DET";
static const char* VP_RUN_PLANE_DET     = ":PlaneSegmentor 4";
static const char* VP_RUN_DOOR_DET      = "DOOR_DET";
static const char* VP_RUN_OBJ_RECOG     = ":ObjectRecognition 1";
static const char* VP_RUN_OBJ_TRAIN     = ":ObjectRecognition 2";
static const char* VP_RUN_OBJ_DET       = ":ObjectDetection 1";
static const char* VP_RUN_OBJ_SURF_DET  = "OBJ_SURF_DET";
static const char* VP_RUN_OBJ_SHAPE_DET = "OBJ_SHAPE_DET";


static const char* VP_RUN_MAPPING       = "MAPPING";
static const char* VP_RUN_FALL_TRAIN    = "FALL_TRAIN";
static const char* VP_RUN_FALL_DET      = "FALL_DET";
static const char* VP_RUN_GES_TRAIN     = "GES_TRAIN";
static const char* VP_RUN_GES_RECOG     = "GES_RECOG";



// For WiW
static const char* VP_ENROLL_SET_NAME   = "ER_NAME";
static const char* VP_CLEAR_PEOPLE      = "CLR_PEOPLE";

// People result status for e.g, Follow-Me, Who-is-Who
static const char* VP_FM_TRACKING       = "FM_Tracking";
static const char* VP_FM_ENROLLING      = "FM_Enrollment";
static const char* VP_WIW_ENROLL        = "WIW_Enroll";
static const char* VP_WIW_REQ_NAME      = "WIW_Request";
static const char* VP_WIW_DET           = "WIW_Detected";
static const char* VP_WIW_RECOGING      = "WIW_Recoging";
static const char* VP_WIW_RECOGNIZED    = "WIW_Recognized";
static const char* VP_WIW_WAVE_DET      = "WIW_Waved";
static const char* VP_WAVE_DET          = "Wave_Detected";
static const char* VP_FACE_DET          = "Face_Detected";
static const char* VP_RUN_FACE_Feature  = "Face_Feature";
static const char* VP_FALL_DET          = "Fall_Detected";
}


#endif
