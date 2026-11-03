#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef HAVE_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace fs = std::filesystem;

namespace {

constexpr double kScale1 = 1.0;
constexpr double kScale2 = 0.75 * kScale1;
constexpr int kOutputWidth = 960;
constexpr int kOutputHeight = 480;
constexpr double kCenterU = 0.5;
constexpr double kCenterV = 0.5;
const fs::path kInputDirectory{"/home/gq/guoqian/Projects-AEB/newCar"};// /app/avm/params/cam
const fs::path kInputDirectory2{"/home/gq/guoqian/Projects-AEB/newCar"};// /data/avm/calibrated
const fs::path kOutputDirectory{"../autoCalibrate"};// /app/avm/aeb
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<std::array<double, 3>, 3>;
using Distortion = std::array<double, 4>;

struct CameraXml {
    Vector3 fpt{};
    double yawAngle{};
    double pitchAngle{};
    double headAngle{};
    std::array<double, 9> cameraMatrix{};
};

struct Calibration {
    double worldX{};
    double worldY{};
    double worldZ{};
    double yaw{};
    double pitch{};
    double roll{};
    double imageWidth{};
    double imageHeight{};
    double focalU{};
    double focalV{};
    double centerU{};
    double centerV{};
    Distortion distortion{};
    Matrix3 activeRotation{};
    Vector3 activeTranslation{};
};

struct CylCalibration {
    double focal{};
    double centerU{};
    double centerV{};
    double horizontalFov{};
    double verticalFov{};
    Matrix3 cameraToCylinder{};
    Matrix3 activeRotation{};
    Vector3 activeTranslation{};
};

struct MapData {
    int width{};
    int height{};
    std::vector<float> u;
    std::vector<float> v;
};

double radians(double degrees) {
    return degrees * std::acos(-1.0) / 180.0;
}

Matrix3 multiply(const Matrix3& left, const Matrix3& right) {
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int index = 0; index < 3; ++index) {
                result[row][column] += left[row][index] * right[index][column];
            }
        }
    }
    return result;
}

Vector3 multiply(const Matrix3& matrix, const Vector3& vector) {
    Vector3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result[row] += matrix[row][column] * vector[column];
        }
    }
    return result;
}

Matrix3 transpose(const Matrix3& matrix) {
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result[row][column] = matrix[column][row];
        }
    }
    return result;
}

Matrix3 rotationX(double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{1.0, 0.0, 0.0},
             {0.0, cosine, -sine},
             {0.0, sine, cosine}}};
}

Matrix3 rotationY(double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{cosine, 0.0, sine},
             {0.0, 1.0, 0.0},
             {-sine, 0.0, cosine}}};
}

Matrix3 rotationZ(double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{cosine, -sine, 0.0},
             {sine, cosine, 0.0},
             {0.0, 0.0, 1.0}}};
}

std::string readFile(const fs::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Unable to open XML file: " + path.string());
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

std::string tagContents(const std::string& xml, const std::string& tag) {
    const std::regex opening("<" + tag + R"((?:\s+[^>]*)?>)");
    std::smatch match;
    if (!std::regex_search(xml, match, opening)) {
        throw std::runtime_error("Missing <" + tag + "> element");
    }
    const std::size_t contentStart =
        static_cast<std::size_t>(match.position() + match.length());
    const std::string closingTag = "</" + tag + ">";
    const std::size_t contentEnd = xml.find(closingTag, contentStart);
    if (contentEnd == std::string::npos) {
        throw std::runtime_error("Missing closing </" + tag + "> element");
    }
    return xml.substr(contentStart, contentEnd - contentStart);
}

std::vector<double> parseNumbers(const std::string& text) {
    static const std::regex number(
        R"([-+]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][-+]?[0-9]+)?)");
    std::vector<double> values;
    for (std::sregex_iterator it(text.begin(), text.end(), number), end; it != end; ++it) {
        values.push_back(std::stod(it->str()));
    }
    return values;
}

std::vector<double> readValues(const std::string& xml, const std::string& tag,
                               std::size_t expectedCount) {
    const auto values = parseNumbers(tagContents(xml, tag));
    if (values.size() != expectedCount) {
        throw std::runtime_error("Expected " + std::to_string(expectedCount) +
                                 " numeric values in <" + tag + ">, got " +
                                 std::to_string(values.size()));
    }
    return values;
}

double readSingleValue(const std::string& xml, const std::string& tag) {
    const auto values = parseNumbers(tagContents(xml, tag));
    if (values.size() != 1) {
        throw std::runtime_error("Expected one numeric value in <" + tag + ">");
    }
    return values.front();
}

CameraXml readCameraXml(const fs::path& path) {
    const std::string xml = readFile(path);
    CameraXml camera;
    const auto fpt = readValues(xml, "fpt", 3);
    std::copy(fpt.begin(), fpt.end(), camera.fpt.begin());
    camera.yawAngle = readSingleValue(xml, "yawangle");
    camera.pitchAngle = readSingleValue(xml, "pitchangle");
    camera.headAngle = readSingleValue(xml, "headangle");
    const auto matrix = readValues(tagContents(xml, "CameraMatrix"), "data", 9);
    std::copy(matrix.begin(), matrix.end(), camera.cameraMatrix.begin());
    return camera;
}

Distortion readDistortion(const fs::path& path) {
    const std::string xml = readFile(path);
    const auto values = readValues(
        tagContents(xml, "distcoef_angle_to_realheight"), "data", 4);
    Distortion result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

Matrix3 activeRotation(const CameraXml& camera) {
    const Matrix3 combined =
        multiply(multiply(rotationZ(radians(camera.headAngle)),
                          rotationY(radians(camera.pitchAngle))),
                 rotationX(radians(camera.yawAngle)));
    return transpose(combined);
}

Calibration makeCalibration(const CameraXml& camera, const Vector3& translation,
                            const Distortion& distortion,
                            const std::array<double, 9>& intrinsics) {
    Calibration calibration;
    calibration.worldX = translation[0];
    calibration.worldY = translation[1];
    calibration.worldZ = translation[2];
    calibration.yaw = camera.headAngle;
    calibration.pitch = camera.pitchAngle;
    calibration.roll = camera.yawAngle;
    calibration.imageWidth = 2.0 * intrinsics[2] / kScale1;
    calibration.imageHeight = 2.0 * intrinsics[5] / kScale1;
    calibration.focalU = intrinsics[0] / kScale1;
    calibration.focalV = intrinsics[4] / kScale1;
    calibration.centerU = intrinsics[2] / kScale1;
    calibration.centerV = intrinsics[5] / kScale1;
    calibration.distortion = distortion;
    calibration.activeRotation = activeRotation(camera);
    calibration.activeTranslation = multiply(calibration.activeRotation, translation);
    for (double& value : calibration.activeTranslation) {
        value = -value;
    }
    return calibration;
}

Matrix3 cameraToCylinder(const Calibration& calibration) {
    return multiply(rotationY(radians(calibration.pitch)),
                    rotationX(radians(calibration.roll)));
}

CylCalibration makeCylCalibration(const Calibration& calibration) {
    CylCalibration cylinder;
    cylinder.cameraToCylinder = cameraToCylinder(calibration);
    cylinder.focal = calibration.focalU * kScale2;
    const Vector3 ray =
        multiply(cylinder.cameraToCylinder, Vector3{1.0, 0.0, 0.0});
    const double horizontalMagnitude = std::hypot(ray[0], ray[1]);
    if (horizontalMagnitude <= 1e-12) {
        throw std::runtime_error(
            "Original camera optical axis is degenerate in cylinder coordinates");
    }
    cylinder.centerU = kOutputWidth * kCenterU +
                       cylinder.focal * std::atan2(-ray[1], ray[0]);
    cylinder.centerV = kOutputHeight * kCenterV -
                       cylinder.focal * ray[2] / horizontalMagnitude;
    cylinder.horizontalFov = kOutputWidth / cylinder.focal * 180.0 / std::acos(-1.0);
    cylinder.verticalFov =
        2.0 * std::atan(kOutputHeight / (2.0 * cylinder.focal)) *
        180.0 / std::acos(-1.0);
    cylinder.activeRotation =
        multiply(cylinder.cameraToCylinder, calibration.activeRotation);
    cylinder.activeTranslation =
        multiply(cylinder.cameraToCylinder, calibration.activeTranslation);
    return cylinder;
}

void writeNumber(std::ostream& output, double value) {
    output << std::setprecision(17) << value;
}

void writeVector(std::ostream& output, const Vector3& values, int indent) {
    output << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) output << ", ";
        writeNumber(output, values[index]);
    }
    output << "]";
    (void)indent;
}

void writeMatrix(std::ostream& output, const Matrix3& matrix, int indent) {
    output << "[\n";
    for (int row = 0; row < 3; ++row) {
        output << std::string(indent + 4, ' ') << "[";
        for (int column = 0; column < 3; ++column) {
            if (column != 0) output << ", ";
            writeNumber(output, matrix[row][column]);
        }
        output << "]" << (row == 2 ? "\n" : ",\n");
    }
    output << std::string(indent, ' ') << "]";
}

void writeArray(std::ostream& output, const Distortion& values) {
    output << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) output << ", ";
        writeNumber(output, values[index]);
    }
    output << "]";
}

void writeFixedModelConfig(std::ostream& output) {
    output << R"json(    "model": {
        "name": "lyl",
        "target_width": 960,
        "target_height": 480,
        "target_type": "NV12",
        "num_cls": 8,
        "num_mask": 32,
        "conf_thresh": 0.5,
        "iou_thresh": 0.3,
        "mask_thresh": 0.5,
        "batch_size": 1,
        "mask_up": 4,
        "id2name": {
            "0": "person",
            "1": "bicycle",
            "2": "car",
            "3": "motorcycle",
            "4": "fake person",
            "5": "bus",
            "6": "fake bicycle",
            "7": "truck"
        },
        "whitelist": {
            "0": {"name": "person", "size": [0.4, 0.4, 1.7]},
            "1": {"name": "bicycle", "size": [1.8, 0.6, 1.1]},
            "2": {"name": "car", "size": [4.5, 1.8, 1.5]},
            "3": {"name": "motorcycle", "size": [2.1, 0.8, 1.2]},
            "4": {"name": "fake person", "size": [0.4, 0.4, 1.7]},
            "5": {"name": "bus", "size": [12.0, 2.5, 3.2]},
            "6": {"name": "fake bicycle", "size": [1.8, 0.6, 1.1]},
            "7": {"name": "truck", "size": [8.0, 2.5, 3.5]}
        }
    },
    "other": {"velocity_mode": "ego"},
    "3d_model": {
        "model_spec": {
            "input_w": 512,
            "input_h": 256,
            "bbox_rows": 4,
            "keypoint_count": 8,
            "classes": [
                {"model_id": 0, "output_id": 2, "name": "car", "color_bgr": [107, 107, 255]},
                {"model_id": 1, "output_id": 0, "name": "person", "color_bgr": [43, 146, 255]},
                {"model_id": 2, "output_id": 1, "name": "bike", "color_bgr": [59, 212, 255]}
            ],
            "ground_first": [0, 1, 2, 3],
            "ground_second": [4, 5, 6, 7]
        },
        "postprocess": {"conf": 0.3, "iou": 0.45, "max_det": 10, "ground": "first"}
    },
    "angle_postproc": {
        "enabled": true,
        "column": 6,
        "input_unit": "rad",
        "scale": 1.0,
        "offset": 0.0,
        "invert": false,
        "wrap_mode": "half_pi",
        "decode_mode": "bins_residual",
        "angle_bins": 8,
        "raw_layout": "auto",
        "raw_logits_first": true
    }
)json";
}

void writeConfig(const fs::path& path, const Calibration& calibration,
                 const CylCalibration& cylinder) {
    fs::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Unable to write config: " + path.string());
    }
    output << "// R-AEB 任务配置文件\n{\n";
    output << "    \"fisheye\": {\n"
           << "        \"type\": \"fisheye\",\n"
           << "        \"image_type\": \"NV12\",\n"
           << "        \"world_x\": ";
    writeNumber(output, calibration.worldX);
    output << ",\n        \"world_y\": ";
    writeNumber(output, calibration.worldY);
    output << ",\n        \"world_z\": ";
    writeNumber(output, calibration.worldZ);
    output << ",\n        \"yaw\": ";
    writeNumber(output, calibration.yaw);
    output << ",\n        \"pitch\": ";
    writeNumber(output, calibration.pitch);
    output << ",\n        \"roll\": ";
    writeNumber(output, calibration.roll);
    output << ",\n        \"angle_type\": \"degree\",\n"
           << "        \"image_width\": ";
    writeNumber(output, calibration.imageWidth);
    output << ",\n        \"image_height\": ";
    writeNumber(output, calibration.imageHeight);
    output << ",\n        \"focal_u\": ";
    writeNumber(output, calibration.focalU);
    output << ",\n        \"focal_v\": ";
    writeNumber(output, calibration.focalV);
    output << ",\n        \"center_u\": ";
    writeNumber(output, calibration.centerU);
    output << ",\n        \"center_v\": ";
    writeNumber(output, calibration.centerV);
    output << ",\n        \"fov\": 204,\n"
           << "        \"pinhole_distort\": [],\n"
           << "        \"fisheye_distort\": ";
    writeArray(output, calibration.distortion);
    output << ",\n        \"R_act\": ";
    writeMatrix(output, calibration.activeRotation, 8);
    output << ",\n        \"t_act\": ";
    writeVector(output, calibration.activeTranslation, 8);
    output << "\n    },\n    \"cyl\": {\n"
           << "        \"type\": \"cyl\",\n"
           << "        \"image_type\": \"NV12\",\n"
           << "        \"image_width\": " << kOutputWidth
           << ",\n        \"image_height\": " << kOutputHeight
           << ",\n        \"focal_u\": ";
    writeNumber(output, cylinder.focal);
    output << ",\n        \"focal_v\": ";
    writeNumber(output, cylinder.focal);
    output << ",\n        \"center_u\": ";
    writeNumber(output, cylinder.centerU);
    output << ",\n        \"center_v\": ";
    writeNumber(output, cylinder.centerV);
    output << ",\n        \"hfov_deg\": ";
    writeNumber(output, cylinder.horizontalFov);
    output << ",\n        \"vfov_deg\": ";
    writeNumber(output, cylinder.verticalFov);
    output << ",\n        \"R_cam_to_cyl\": ";
    writeMatrix(output, cylinder.cameraToCylinder, 8);
    output << ",\n        \"R_cyl_to_cam\": ";
    writeMatrix(output, transpose(cylinder.cameraToCylinder), 8);
    output << ",\n        \"R_act\": ";
    writeMatrix(output, cylinder.activeRotation, 8);
    output << ",\n        \"t_act\": ";
    writeVector(output, cylinder.activeTranslation, 8);
    output << "\n    },\n";
    writeFixedModelConfig(output);
    output << "}\n";
    if (!output) {
        throw std::runtime_error("Failed while writing config: " + path.string());
    }
}

double distortRadius(double theta, const Distortion& distortion) {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    return theta * (1.0 + distortion[0] * theta2 + distortion[1] * theta4 +
                    distortion[2] * theta6 + distortion[3] * theta8);
}

Vector3 projectCylinderPoint(double horizontal, double vertical,
                             int width, int height, double focal,
                             const Matrix3& cylToCamera,
                             const Calibration& calibration) {
    const double phi = (horizontal - width * kCenterU) / focal;
    const double heightAngle = (height * kCenterV - vertical) / focal;
    const Vector3 direction = multiply(
        cylToCamera, Vector3{std::cos(phi), -std::sin(phi), heightAngle});
    const double perpendicular = std::hypot(direction[1], direction[2]);
    const double theta = std::atan2(perpendicular, direction[0]);
    const double distance = distortRadius(theta, calibration.distortion);
    const double inversePerpendicular =
        perpendicular < 1e-6 ? 0.0 : 1.0 / perpendicular;
    const double uScaled =
        calibration.centerU * kScale2 -
        calibration.focalU * kScale2 * distance * direction[1] *
            inversePerpendicular;
    const double vScaled =
        calibration.centerV * kScale2 -
        calibration.focalV * kScale2 * distance * direction[2] *
            inversePerpendicular;
    return {uScaled / kScale2, vScaled / kScale2, direction[0]};
}

MapData makeMap(const Calibration& calibration) {
    MapData map;
    map.width = kOutputWidth;
    map.height = kOutputHeight;
    const std::size_t count =
        static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
    map.u.resize(count);
    map.v.resize(count);
    const double focal = calibration.focalU * kScale2;
    const Matrix3 cylToCamera = transpose(cameraToCylinder(calibration));
    for (int row = 0; row < map.height; ++row) {
        for (int column = 0; column < map.width; ++column) {
            const Vector3 source = projectCylinderPoint(
                column, row, map.width, map.height, focal, cylToCamera, calibration);
            const std::size_t index =
                static_cast<std::size_t>(row) * map.width + column;
            map.u[index] = static_cast<float>(source[0]);
            map.v[index] = static_cast<float>(source[1]);
        }
    }
    return map;
}

void saveMap(const fs::path& path, const MapData& map) {
    fs::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Unable to write map: " + path.string());
    }
    output.write(reinterpret_cast<const char*>(map.u.data()),
                 static_cast<std::streamsize>(map.u.size() * sizeof(float)));
    output.write(reinterpret_cast<const char*>(map.v.data()),
                 static_cast<std::streamsize>(map.v.size() * sizeof(float)));
    if (!output) {
        throw std::runtime_error("Failed while writing map: " + path.string());
    }
}

std::int32_t matlabRound(double value) {
    return static_cast<std::int32_t>(std::round(value));
}

void saveMesh(const fs::path& path, const Calibration& calibration,
              int width, int height, int m = 4) {
    if (m < 0 || m > 16) {
        throw std::runtime_error("Mesh subdivision exponent must be between 0 and 16");
    }
    fs::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Unable to write mesh: " + path.string());
    }
    const int step = 1 << m;
    const double focal = calibration.focalU * kScale2;
    const Matrix3 cylToCamera = transpose(cameraToCylinder(calibration));
    for (int row = 0; row <= height; row += step) {
        for (int column = 0; column <= width; column += step) {
            const Vector3 source = projectCylinderPoint(
                column, row, width, height, focal, cylToCamera, calibration);
            const auto deltaU = matlabRound((source[0] - column) * 8.0);
            const auto deltaV = matlabRound((source[1] - row) * 8.0);
            output << deltaU << ' ' << deltaV << '\n';
        }
    }
    if (!output) {
        throw std::runtime_error("Failed while writing mesh: " + path.string());
    }
}

#ifdef HAVE_OPENCV
void rectifyImages(const fs::path& imageDirectory, const fs::path& outputDirectory,
                   const Calibration& calibration, const MapData& map) {
    if (!fs::exists(imageDirectory)) {
        std::cerr << "Image directory not found; skipping rectification: "
                  << imageDirectory << '\n';
        return;
    }
    fs::create_directories(outputDirectory);
    cv::Mat mapU(map.height, map.width, CV_32FC1,
                 const_cast<float*>(map.u.data()));
    cv::Mat mapV(map.height, map.width, CV_32FC1,
                 const_cast<float*>(map.v.data()));
    for (const auto& entry : fs::directory_iterator(imageDirectory)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".jpg") {
            continue;
        }
        const cv::Mat input = cv::imread(entry.path().string(), cv::IMREAD_COLOR);
        if (input.empty()) {
            throw std::runtime_error("Failed to read image: " + entry.path().string());
        }
        cv::Mat resized;
        cv::resize(input, resized,
                   cv::Size(static_cast<int>(calibration.imageWidth),
                            static_cast<int>(calibration.imageHeight)),
                   0.0, 0.0, cv::INTER_LINEAR);
        cv::Mat rectified;
        cv::remap(resized, rectified, mapU, mapV, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::Mat valid(resized.rows, resized.cols, CV_8UC1, cv::Scalar(255));
        cv::Mat validMapped;
        cv::remap(valid, validMapped, mapU, mapV, cv::INTER_NEAREST,
                  cv::BORDER_CONSTANT, cv::Scalar(0));
        rectified.setTo(cv::Scalar(0, 0, 0), validMapped == 0);
        const fs::path outputPath = outputDirectory / entry.path().filename();
        if (!cv::imwrite(outputPath.string(), rectified)) {
            throw std::runtime_error("Failed to write rectified image: " +
                                     outputPath.string());
        }
        std::cout << "Saved rectified image: " << outputPath << '\n';
    }
}
#endif

void generateCamera(const std::string& name, const Calibration& calibration,
                    const fs::path& root) {
    const CylCalibration cylinder = makeCylCalibration(calibration);
    const MapData map = makeMap(calibration);
    const fs::path mapDirectory = root / "maps" / name;
    writeConfig(root / (name == "front" ? "configFront.jsonc" : "configRear.jsonc"),
                calibration, cylinder);
    saveMap(mapDirectory / (name + "_visper_960x480_uv.bin"), map);
    saveMesh(mapDirectory / "mesh4.txt", calibration, map.width, map.height);
#ifdef HAVE_OPENCV
    rectifyImages(root / name, root / (name == "front" ? "outF" : "outR"),
                  calibration, map);
#endif
    std::cout << "Generated " << name << " config, mesh4.txt, and UV bin\n";
}

}  // namespace

int main() {
    try {
        const fs::path frontPath = kInputDirectory2 / "front_3d_map.xml";
        const fs::path rearPath = kInputDirectory2 / "rear_3d_map.xml";
        const fs::path intrinsicsPath = kInputDirectory / "cameraInterParams.xml";
        const CameraXml front = readCameraXml(frontPath);
        const CameraXml rear = readCameraXml(rearPath);
        const Distortion distortion = readDistortion(intrinsicsPath);

        const Vector3 frontTranslation{
            (front.fpt[1] + std::abs(rear.fpt[1]) - 100.0) * 0.01,
            front.fpt[0] * 0.01,
            front.fpt[2] * 0.01};
        const Vector3 rearTranslation{
            -1.0, rear.fpt[0] * 0.01, rear.fpt[2] * 0.01};

        const Calibration frontCalibration =
            makeCalibration(front, frontTranslation, distortion, front.cameraMatrix);
        const Calibration rearCalibration =
            makeCalibration(rear, rearTranslation, distortion, front.cameraMatrix);

        generateCamera("front", frontCalibration, kOutputDirectory);
        generateCamera("rear", rearCalibration, kOutputDirectory);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "generateJsonFromXml: " << error.what() << '\n';
        return 1;
    }
}
