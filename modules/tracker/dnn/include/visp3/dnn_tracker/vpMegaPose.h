#ifndef VP_MEGAPOSE_H
#define VP_MEGAPOSE_H

#ifdef VISP_HAVE_NLOHMANN_JSON


#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpRect.h>


#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * Result from a pose estimation performed by megapose
 * Contains:
 * - the estimated pose as a vpHomogeneousMatrix
 * - The confidence score between 0-1.
 * This score is defined as 1 if the current pose is in the basin of attraction of the true pose
 * (whether the Megapose refiner can converge to the true pose). As such, it should be used to detect divergence.
 * - The bounding box of the object in image space.
*/
class vpMegaPoseEstimate {
public:
    vpMegaPoseEstimate() {}
    vpHomogeneousMatrix cTo;
    double score;
    vpRect boundingBox;
};

inline void to_json(json& j, const vpRect& d)
{
    std::vector<double> values = {
        d.getLeft(), d.getTop(), d.getRight(), d.getBottom()
    };
    j = values;
}
inline void from_json(const json& j, vpRect& d)
{
    std::vector<double> values = j.get<std::vector<double>>();
    assert((values.size() == 4));
    d.setLeft(values[0]);
    d.setTop(values[1]);
    d.setRight(values[2]);
    d.setBottom(values[3]);
}

inline void from_json(const json& j, vpMegaPoseEstimate& m)
{
    m.score = j["score"];
    m.cTo = j["cTo"];
    m.boundingBox = j["boundingBox"];
}

/**
* Class to communicate with a Megapose server.
Megapose is a deep learning-based method to estimate the 6D pose of novel objects, meaning that it does not require training for a specific object.
Megapose is a multistage method and can be used either as a full pose estimation algorithm or as a tracker.
Megapose works by render and compare: a synthetized view of an object is compared with a real-world image to see if the poses match.

Behind the scene, there are two main models:
    - The coarse model: given an image, multiple synthetic view are compared with the "true" object image and a model outputs the probability of a render corresponding to the same object as the true image. This model requires the object to be detected (bounding box) in the image.
    - The refiner model: Given an initial pose estimate, this model predicts a pose displacement to best align render and true image. It is called iteratively, for a fixed number of iterations.

*/
class vpMegaPose {
public:
    /**
    * Enum to communicate with python server.
    * Used to map to a string, which the client and server check to see what message they are getting/expecting.
    */
    enum ServerMessage {
        UNKNOWN = 0,
        ERR = 1, // An error occurred server side
        OK = 2, // All good, no return
        GET_POSE = 3,
        RET_POSE = 4,
        GET_VIZ = 5,
        RET_VIZ = 6,
        SET_INTR = 7,
        GET_SCORE = 8,
        RET_SCORE = 9,
        SET_SO3_GRID_SIZE = 10,
    };
    /**
    * Instanciates a connection to a megapose server.
    * The server should already be started  and listening at host:port.
    * \param host The host to connect to (IP address)
    * \param port The port on which the server is listening for incoming connections
    * \param cam Intrinsics of the camera with which the images sent to the megapose server are acquired
    * \param height Height of the images sent to the server
    * \param width Width of the images sent to the server
    */
    vpMegaPose(const std::string& host, int port, const vpCameraParameters& cam, unsigned height, unsigned width);


    /**
    * Estimate the poses of objects (in the frame of the camera c) with Megapose.
    * The object origins used to estimate the poses are those used by the megapose server.
    * \param image the image, acquired by camera c, used to estimate the pose of the objects.
    * \param depth an optional depth image, that must be aligned with the RGB image. If provided, the megapose server should be configure to use the depth.
    *               Note that the using depth may lead to a noisy estimation and should not always be preferred.
    * \param depth_to_m a scale factor that is used to convert the depth map into meters. If depth is null, the value is ignored.
    * \param detections the objects detected in the image. Used only for the coarse model, which uses the size of the bounding box to generate initial guesses.
    *                   The label associated to an object should match the name of an object known by the megapose server.
    * \param initial_cTos An optional initial pose estimate for each object present in the image. If not null, then the megapose server only runs the refiner model,
    *                    which is faster but only allows for smaller corrections (depending on the number of refiner iterations)
    * \param refiner_iterations Number of megapose refiner iterations to be performed.
    *
    * \return a list of vpMegaPoseEstimate, one for each input detection, (same length as objectNames)
    */
    std::vector<vpMegaPoseEstimate> estimatePoses(const vpImage<vpRGBa>& image, const std::vector<std::string>& objectNames,
        const vpImage<uint16_t>* const depth = nullptr, const double depth_to_m = 0.f,
        const std::vector<vpRect>* const boundingBoxes = nullptr,
        const std::vector<vpHomogeneousMatrix>* const initial_cTos = nullptr,
        int refinerIterations = -1);
    /**
    * Score the input poses with Megapose. The score, between 0 and 1, indicates whether the refiner model can converge to the correct pose.
    * As such, it should mainly be used to detect divergence, and not the quality of the pose estimate.
    *
    * \param image The input RGB image in which the objects are located
    * \param objectNames the detected objects. Names should be known by the megapose server
    * \param cTos the object poses to be scored.
    *
    * \return a list of scores, each between 0 and 1. Returns one score per object (the result has the same length as objectNames)
    */
    std::vector<double> scorePoses(const vpImage<vpRGBa>& image, const std::vector<std::string>& objectNames,
        const std::vector<vpHomogeneousMatrix>& cTos);

    /**
    * Set the camera parameters for the megapose server
    * \param cam The camera intrinsics
    * \param height the incoming images' height
    * \param width the incoming images' width
    * Note that the height and width should be equal to the width of the images used in estimatePoses or scorePoses. Otherwise an exception will be thrown.
    */
    void setIntrinsics(const vpCameraParameters& cam, unsigned height, unsigned width);

    vpImage<vpRGBa> viewObjects(const vpImage<vpRGBa>& baseImage, const std::vector<std::string>& objectNames,
        const std::vector<vpHomogeneousMatrix>& poses, const std::string& viewType);

    /*
    * Set the number of renders used for coarse pose estimation by megapose.
    */
    void setCoarseNumSamples(const unsigned num);

    ~vpMegaPose();

private:
    // Server connection data
    int serverSocket;
    int fd;

    std::mutex mutex; // Since client-server communications are synchronous, avoid multiple parallel communications

    void makeMessage(const vpMegaPose::ServerMessage messageType, std::vector<uint8_t>& data) const;
    std::pair<vpMegaPose::ServerMessage, std::vector<uint8_t>> readMessage() const;

    const static std::unordered_map<vpMegaPose::ServerMessage, std::string> codeMap;
    static std::string messageToString(const vpMegaPose::ServerMessage messageType);
    static vpMegaPose::ServerMessage stringToMessage(const std::string& s);
};

#endif
#endif