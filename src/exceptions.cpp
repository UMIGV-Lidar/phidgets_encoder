#include "exceptions.h"

#include <phidgets_api/phidget.h> // phidgets::Phidget

#include <utility> // std::move

umigv::UnableToConnectException::UnableToConnectException(const int error)
    : std::runtime_error{ phidgets::Phidget::getErrorDescription(error) },
      error_{ error }
{ }

umigv::UnableToConnectException::UnableToConnectException(
    const int error, const std::string &what
) : std::runtime_error{ what }, error_{ error } { }

umigv::UnableToConnectException::UnableToConnectException(
    const int error, const char *what
) : std::runtime_error{ what }, error_{ error } { }

int umigv::UnableToConnectException::code() const noexcept {
    return error_;
}

const char* umigv::UnableToConnectException::what() const noexcept {
    return std::runtime_error::what();
}

umigv::DeviceDetachedException::DeviceDetachedException(
    const std::string &what
) : std::runtime_error{ what } { }

umigv::DeviceDetachedException::DeviceDetachedException(
    const char *what
) : std::runtime_error{ what } { }

const char* umigv::DeviceDetachedException::what() const noexcept {
    return std::runtime_error::what();
}

umigv::ParameterNotFoundException::ParameterNotFoundException(
    const std::string &what, std::string parameter
) : std::runtime_error{ what }, parameter_{ std::move(parameter) } { }

umigv::ParameterNotFoundException::ParameterNotFoundException(
    const std::string &what, const char *const parameter
) : std::runtime_error{ what }, parameter_{ parameter } { }

umigv::ParameterNotFoundException::ParameterNotFoundException(
    const char *const what, std::string parameter
) : std::runtime_error{ what }, parameter_{ std::move(parameter) } { }

umigv::ParameterNotFoundException::ParameterNotFoundException(
    const char *const what, const char *const parameter
) : std::runtime_error{ what }, parameter_{ parameter } { }

const char* umigv::ParameterNotFoundException::what() const noexcept {
    return std::runtime_error::what();
}

const char* umigv::ParameterNotFoundException::parameter() const noexcept {
    return parameter_.c_str();
}
