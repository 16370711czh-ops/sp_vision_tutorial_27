#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

int main()
{
    Camera camera;

    auto_aim::YOLO yolo("./configs/yolo.yaml");

    cv::Mat img;

    while (true)
    {
        camera.read(img);

        if (img.empty())
            continue;

        auto armors = yolo.detect(img);

        for (auto& armor : armors)
        {
            tools::draw_points(
                img,
                armor.points,
                cv::Scalar(0, 255, 0),
                2
            );

            auto info = fmt::format(
                "{:.2f} {} {} {}",
                armor.confidence,
                auto_aim::COLORS[armor.color],
                auto_aim::ARMOR_NAMES[armor.name],
                auto_aim::ARMOR_TYPES[armor.type]
            );

            tools::draw_text(
                img,
                info,
                armor.center,
                cv::Scalar(0, 255, 0),
                1.0,
                2
            );
        }

        cv::imshow("img", img);

        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
