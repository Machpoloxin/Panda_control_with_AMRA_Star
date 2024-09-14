#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "AMRA_Star/amra_star.hpp"




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("amra_star");
    std::string map_path = package_share_directory + "/maps/Testmap.map";


    std::array<int, 3> start = {12, 9, 7};
    std::array<int, 3> new_start = {12, 10, 8};
    std::array<double, 4> start_ori = {0.707, 0.707, 0, 0};
    std::array<int, 3> goal = {40, 10, 8};
    std::array<double, 4> goal_ori = {0.707, 0, 0.707, 0};

    AMRA_Star::AMRAstar amraStar(10, 10, map_path, start, start_ori, goal, goal_ori, 1, 100);
    std::shared_ptr<AMRA_Star::AMRAstar> amra_star_ = std::make_shared<AMRA_Star::AMRAstar>(10, 10, map_path, start, start_ori, goal, goal_ori, 1, 100);
    amraStar.search();
    amraStar.displaySolution();
    std::cout<<"Restart:"<<std::endl;
    amraStar.reSearchPath(new_start,start_ori,goal,goal_ori);
    amraStar.displaySolution();
    std::cout<<"Test Solution:"<<std::endl;

    std::vector<AMRA_Star::Node> my_solution = amraStar.getSolution();
    for (const auto& step : my_solution) {
            std::cout << '(' << step.position[0] << ',' << step.position[1] << ',' << step.position[2] << ')';
            step.orientation.print();
    }
    std::cout<<"Test ptr Solution:"<<std::endl;
    amra_star_->search();
    amra_star_->displaySolution();
    std::cout<<"Restart ptr:"<<std::endl;

    amra_star_->reSearchPath(new_start,start_ori,goal,goal_ori);
    amra_star_->displaySolution();
    std::cout<<"Test ptr Solution:"<<std::endl;

    std::vector<AMRA_Star::Node> my_ptr_solution = amra_star_->getSolution();
    for (const auto& step : my_ptr_solution) {
            std::cout << '(' << step.position[0] << ',' << step.position[1] << ',' << step.position[2] << ')';
            step.orientation.print();
    }

    rclcpp::shutdown();
    return 0;
}
