#ifndef NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
#define NAV2_CORE__PLANNER_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

class PlannerException : public std::runtime_error
{
  public:
    explicit PlannerException(const std::string & description)
    : std::runtime_error(description) {}
};

class StartOutsideMapBounds : public PlannerException
{
  public:
    explicit StartOutsideMapBounds(const std::string & description)
    : PlannerException(description) {}
};

class GoalOutsideMapBounds : public PlannerException
{
  public:
    explicit GoalOutsideMapBounds(const std::string & description)
    : PlannerException(description) {}
};

class StartOccupied : public PlannerException
{
  public:
    explicit StartOccupied(const std::string & description)
    : PlannerException(description) {}
};

class GoalOccupied : public PlannerException
{
  public:
    explicit GoalOccupied(const std::string & description)
    : PlannerException(description) {}
};

class NoValidPathCouldBeFound : public PlannerException
{
  public:
    explicit NoValidPathCouldBeFound(const std::string & description)
    : PlannerException(description) {}
};

}

#endif