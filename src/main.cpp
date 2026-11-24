#include <swarm_planner/offline/offline_map_planner.hpp>

#include <nlohmann/json.hpp>
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

using json = nlohmann::json;
using namespace swarm_planner;
using namespace swarm_planner::offline;

namespace
{
GridPosition locate(const MultiMapBundle &bundle, const json &pose,
                    const std::string &who, double radius)
{
    const auto name = pose.at("frame_id").get<std::string>();
    const auto x = pose.at("pos").at(0).get<double>();
    const auto y = pose.at("pos").at(1).get<double>();
    std::ostringstream context;
    context << who << "，地图 " << name << "，位置 (" << std::fixed
            << std::setprecision(3) << x << ", " << y << ")m";
    const auto found = bundle.maps.find(name);
    if (found == bundle.maps.end())
        throw std::runtime_error(context.str() + "：未知地图");
    const auto &map = found->second;
    // Reject out-of-range inputs before local_to_grid narrows them to integers.
    const double dx = x - map.origin_x, dy = y - map.origin_y;
    const double c = std::cos(map.origin_yaw), s = std::sin(map.origin_yaw);
    const double gx = (c * dx + s * dy) / map.resolution;
    const double gy = (-s * dx + c * dy) / map.resolution;
    context << "，栅格 (" << std::floor(gx) << ", " << std::floor(gy) << ")";
    if (!(gx >= 0 && gx < map.width && gy >= 0 && gy < map.height))
        throw std::runtime_error(context.str() + "：超出地图范围");
    const auto cell = map.local_to_grid(x, y);
    if (!bundle.traversable(cell) || map.clearance(cell.x, cell.y) < radius)
        throw std::runtime_error(context.str() + "：落入不可通行栅格或净空不足");
    return cell;
}

struct RouteSpace
{
    GridPosition start, finish;
    std::set<GridPosition> cells;
    std::set<std::string> resources;
};

bool near(const std::set<GridPosition> &path, const GridPosition &point, int margin)
{
    for (int dx = -margin; dx <= margin; ++dx)
        for (int dy = -margin; dy <= margin; ++dy)
            if (path.count({point.map_id, point.x + dx, point.y + dy}))
                return true;
    return false;
}

bool intersects(const RouteSpace &a, const RouteSpace &b, int margin)
{
    for (const auto &id : a.resources)
        if (b.resources.count(id))
            return true;
    return std::any_of(b.cells.begin(), b.cells.end(),
                       [&](const auto &cell) { return near(a.cells, cell, margin); });
}

void serialize_conflicts(const MultiMapBundle &bundle,
                         const std::vector<MappedRobot> &robots,
                         const std::vector<RouteSpace> &space, double radius, json &routes)
{
    std::set<std::string> maps;
    std::set<std::size_t> pending;
    for (std::size_t i = 0; i < robots.size(); ++i)
    {
        if (!routes[i]["task_ids"].empty())
            pending.insert(i);
        for (const auto &p : space[i].cells)
            maps.insert(p.map_id);
    }
    if (pending.empty() || robots.size() < 2)
        return;
    if (maps.size() > 1)
        throw std::runtime_error("多设备跨地图结果缺少基于实际到达的 ROOT 空间执行约束");
    const auto &map = bundle.map(*maps.begin());
    const double requested_margin = std::ceil(2 * radius / map.resolution);
    if (requested_margin > 128)
        throw std::runtime_error("路线空间冲突校验超出净空搜索预算");
    const int margin = std::max(1, static_cast<int>(requested_margin));
    std::vector<std::set<std::size_t>> before(robots.size()), conflicts(robots.size());
    for (const auto a : pending)
    {
        for (std::size_t b = 0; b < robots.size(); ++b)
        {
            const auto &x = space[a];
            const auto &y = space[b];
            if (!pending.count(b))
            {
                if (near(x.cells, y.start, margin))
                    throw std::runtime_error("路线 " + robots[a].id + " 与停车设备 " + robots[b].id + " 冲突");
                continue;
            }
            if (b <= a || !intersects(x, y, margin))
                continue;
            conflicts[a].insert(b);
            conflicts[b].insert(a);
            const bool a_first = !near(x.cells, y.start, margin) && !near(y.cells, x.finish, margin);
            const bool b_first = !near(y.cells, x.start, margin) && !near(x.cells, y.finish, margin);
            if (!a_first && !b_first)
                throw std::runtime_error("路线安全顺序无法确定：" + robots[a].id + " 与 " + robots[b].id);
            if (!a_first)
                before[a].insert(b);
            if (!b_first)
                before[b].insert(a);
        }
    }
    while (!pending.empty())
    {
        const auto ready = std::find_if(pending.begin(), pending.end(),
                                       [&](auto i) { return before[i].empty(); });
        if (ready == pending.end())
            throw std::runtime_error("路线依赖成环，无法按实际到达安全启动");
        const auto finished = *ready;
        pending.erase(ready);
        for (const auto next : pending)
        {
            before[next].erase(finished);
            if (conflicts[next].count(finished))
                routes[next]["dependencies"].push_back(robots[finished].id);
        }
    }
}

json plan(const json &request, const std::shared_ptr<const MultiMapBundle> &bundle,
          double radius)
{
    const auto &mission = request.at("mission");
    std::vector<MappedRobot> robots;
    std::vector<MappedTask> tasks;
    std::map<std::string, json> locations;
    std::string errors;
    for (const auto &[id, value] : request.at("robots").items())
    {
        try
        {
            auto start = locate(*bundle, value.at("pose"), "规划起点无效：" + id, radius);
            const double speed = value.at("speed").get<double>();
            if (!std::isfinite(speed) || speed <= 0)
                throw std::runtime_error(id + "：速度必须是有限正数");
            MappedRobot robot;
            robot.id = id;
            robot.start = start;
            robot.capabilities = value.at("capabilities").get<CapabilitySet>();
            robot.clearance_radius_m = radius;
            robot.footprint_radius_m = radius;
            robot.nominal_speed_mps = speed;
            robots.push_back(std::move(robot));
        }
        catch (const std::exception &error)
        {
            if (!errors.empty())
                errors += "；";
            errors += error.what();
        }
    }
    if (!errors.empty())
        throw std::runtime_error(errors);
    json unassignable = json::object();
    for (const auto &task : mission.at("tasks"))
    {
        const auto id = task.at("id").get<std::string>();
        const auto &location = task.at("location");
        const auto cell = locate(*bundle, location, "任务点无效：" + id, 0);
        const auto requirements = task.at("requirements").get<CapabilitySet>();
        const bool capable = std::any_of(robots.begin(), robots.end(), [&](const auto &robot) {
            return std::includes(robot.capabilities.begin(), robot.capabilities.end(),
                                 requirements.begin(), requirements.end());
        });
        if (!capable)
        {
            unassignable[id] = {{"requirements", requirements}, {"reason", "所需检测设备当前不可用"}};
            continue;
        }
        const double seconds = std::ceil(task.at("service_seconds").get<double>());
        if (!(seconds >= 0 && seconds <= std::numeric_limits<int>::max() / 100))
            throw std::runtime_error("任务耗时超出规划范围：" + id);
        tasks.push_back(make_mapped_task(id, cell, requirements,
                                       task.value("task_type", "inspection"),
                                       static_cast<int>(seconds),
                                       task.value("high_priority", false)));
        locations.emplace(id, location);
    }
    const auto plan = OfflineMissionPlanner(MultiMapPathPlanner(bundle)).plan(robots, tasks, !tasks.empty());
    auto routes = json::array();
    std::vector<RouteSpace> space;
    for (std::size_t i = 0; i < robots.size(); ++i)
    {
        auto checkpoints = json::array(), ids = json::array();
        const auto start = robots[i].start;
        RouteSpace occupied{start, start, {start}, {}};
        if (!tasks.empty())
        {
            for (const auto &step : plan.schedules.at(i))
            {
                occupied.cells.insert(step.position);
                if (!step.resource.empty())
                    occupied.resources.insert(step.resource);
                occupied.finish = step.position;
            }
            for (const auto &point : plan.navigation_checkpoints.at(i))
            {
                using Kind = NavigationCheckpointType;
                if (point.type == Kind::Start || point.type == Kind::Finish)
                    continue;
                if (point.type == Kind::Task)
                {
                    ids.push_back(point.task_id);
                    checkpoints.push_back({{"type", "task"}, {"location", locations.at(point.task_id)},
                                           {"task_ids", {point.task_id}}});
                    continue;
                }
                const auto pose = bundle->map(point.position.map_id).grid_to_local(point.position);
                const bool holding = point.type == Kind::Holding;
                json item = {{"type", holding ? "holding" : "turn"},
                             {"location", {{"frame_id", pose.map_id}, {"pos", {pose.x, pose.y, pose.z}}}},
                             {"task_ids", json::array()}};
                if (holding)
                    item["duration"] = (point.departure_tick - point.arrival_tick) * plan.time_step_seconds;
                checkpoints.push_back(std::move(item));
            }
        }
        if (ids.empty())
            checkpoints.clear();
        space.push_back(std::move(occupied));
        routes.push_back({{"robot_id", robots[i].id}, {"checkpoints", std::move(checkpoints)},
                          {"task_ids", std::move(ids)}, {"dependencies", json::array()}});
    }
    serialize_conflicts(*bundle, robots, space, radius, routes);
    return {{"routes", routes}, {"unassignable", unassignable},
            {"scheduling", "sealed-result-dependencies"},
            {"statistics", {{"maximum_load_seconds", plan.maximum_load_ticks * plan.time_step_seconds},
                            {"total_load_seconds", plan.total_load_ticks * plan.time_step_seconds},
                            {"allocation_budget_exhausted", plan.allocation_stats.budget_exhausted}}}};
}
} // namespace

int main()
{
    const auto started = std::chrono::steady_clock::now();
    json result;
    try
    {
        const auto request = json::parse(std::cin);
        MapLoadOptions options;
        // Keep map packages read-only. The full planning budget includes preprocessing.
        options.persistent_cache = false;
        if (request.contains("free_thresh"))
            options.free_thresh_override = request.at("free_thresh").get<double>();
        const auto radius = request.at("radius").get<double>();
        if (!std::isfinite(radius) || radius < 0)
            throw std::runtime_error("invalid footprint radius");
        const auto bundle = MapBundleLoader::load(request.at("map_package").get<std::string>(), options);
        if (request.value("preflight", false))
            result = {{"success", true}};
        else
            result = {{"success", true}, {"plan", plan(request, bundle, radius)}};
    }
    catch (const std::exception &error)
    {
        result = {{"success", false}, {"error", error.what()}};
    }
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    result["metrics"] = {
        {"compute_seconds", std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()},
        {"peak_rss_kib", usage.ru_maxrss}};
    std::cout << result.dump(-1, ' ', false, json::error_handler_t::replace) << '\n';
}
