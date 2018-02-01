#ifndef UMIGV_ROS_TRAITS_HPP
#define UMIGV_ROS_TRAITS_HPP

#include <map> // std::map
#include <string> // std::string
#include <type_traits> // std::true_type, std::false_type
#include <vector> // std::vector

namespace umigv {
    
template <typename T>
struct IsParameter : public std::false_type { };

template <>
struct IsParameter<bool> : public std::true_type { };

template <>
struct IsParameter<int> : public std::true_type { };

template <>
struct IsParameter<float> : public std::true_type { };

template <>
struct IsParameter<double> : public std::true_type { };

template <>
struct IsParameter<std::string> : public std::true_type { };

template <typename T>
struct IsParameter<std::vector<T>> : public IsParameter<T> { };

template <typename T>
struct IsParameter<std::map<std::string, T>> : public IsParameter<T> { };

template <typename T>
constexpr bool IsParameterV = IsParameter<T>::value;

} // namespace umigv

#endif
