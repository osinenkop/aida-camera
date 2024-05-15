#include "JabraManager.hxx"
#include <iostream>
#include <algorithm>
#include <regex>

std::string getNodeName(int argc, char* argv[]){
    const char* target_name{"Jabra"};
    char* entry{};
    std::for_each(argv + 1, argv + argc, [target_name, &entry](char* arg)
    {if(std::strstr(arg, target_name) != nullptr) entry = arg;});
    std::regex pattern(std::string(target_name) + "(.*)");
    std::cmatch match;
    return (std::regex_search(entry, match, pattern))?match[0].str():"";
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    std::string node_name(getNodeName(argc, argv));

    std::shared_ptr<Jabra> j{std::make_shared<Jabra>(std::string(node_name))};
  try {
    j->warmUp();
    j->loopForEver();
    std::clog << "spin" << std::endl;
    rclcpp::spin(j);
  } catch (std::exception &e) {
    std::cerr << "Caught Error " << e.what() << std::endl;
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}