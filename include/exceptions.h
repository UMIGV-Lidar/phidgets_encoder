#ifndef UMIGV_EXCEPTIONS_H
#define UMIGV_EXCEPTIONS_H

#include <stdexcept>
#include <string>

namespace umigv {

class UnableToConnectException : public std::runtime_error {
public:
    explicit UnableToConnectException(int error);

    UnableToConnectException(int error, const std::string &what);

    UnableToConnectException(int error, const char *what);

    virtual ~UnableToConnectException() = default;

    int code() const noexcept;

    virtual const char* what() const noexcept;

private:
    int error_;
};

class DeviceDetachedException : public std::runtime_error {
public:
    explicit DeviceDetachedException(const std::string &what);

    explicit DeviceDetachedException(const char *what);

    virtual ~DeviceDetachedException() = default;

    virtual const char* what() const noexcept;
};

class ParameterNotFoundException : public std::runtime_error {
public:
    ParameterNotFoundException(const std::string &what,
                               std::string parameter);

    ParameterNotFoundException(const std::string &what, const char *parameter);

    ParameterNotFoundException(const char *what, std::string parameter);

    ParameterNotFoundException(const char *what, const char *parameter);

    virtual ~ParameterNotFoundException() = default;

    virtual const char* what() const noexcept;

    const char* parameter() const noexcept;

private:
    std::string parameter_;
};

} // namespace umigv

#endif
