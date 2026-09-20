#include <gtest/gtest.h>

#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/terminal_observation.h>

TEST(TEBBasic, autoResizeLargeValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a large timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResizeSmallValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a small timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResize)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // modify the timediff in the middle and add a pose with a smaller timediff as the last one
  teb.TimeDiff(5) = dt + 2*dt_hysteresis;
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}


TEST(TEBBasic, terminalObservationKeepsSnapshotWhileRoutePhaseIsPending)
{
  teb_local_planner::TerminalObservation::Epoch epoch;
  epoch.event_sequence = 4;
  epoch.plan_sequence = 2;
  epoch.compute_sequence = 9;
  epoch.time_ns = 100;
  teb_local_planner::TerminalObservation::Inputs inputs;
  inputs.pose_frame = "map";
  inputs.goal_frame = "map";
  inputs.velocity_frame = "base_link";
  inputs.pose_stamp_ns = 90;
  inputs.goal_stamp_ns = 80;
  inputs.pose = {{1.0, 2.0, 0.2}};
  inputs.goal = {{1.1, 2.0, 0.2}};
  inputs.velocity = {{0.01, 0.02, 0.0}};
  inputs.position_error = 0.1;
  inputs.yaw_error = 0.0;
  inputs.position_tolerance = 0.2;
  inputs.yaw_tolerance = 0.1;
  inputs.route_phase_pending = true;
  inputs.translational_stopped_limit = 0.05;
  inputs.rotational_stopped_limit = 0.05;
  inputs.via_count = -1;

  const auto observation = teb_local_planner::TerminalObservation::evaluate(
      epoch, inputs, true);

  EXPECT_TRUE(observation.evaluated);
  EXPECT_EQ(observation.epoch.compute_sequence, 9u);
  EXPECT_EQ(observation.inputs.pose_stamp_ns, 90u);
  EXPECT_EQ(observation.inputs.goal_stamp_ns, 80u);
  EXPECT_EQ(observation.inputs.velocity_frame, "base_link");
  EXPECT_DOUBLE_EQ(observation.inputs.pose[0], 1.0);
  EXPECT_DOUBLE_EQ(observation.inputs.velocity[1], 0.02);
  EXPECT_TRUE(observation.position_ok);
  EXPECT_TRUE(observation.yaw_ok);
  EXPECT_FALSE(observation.route_phase_ok);
  EXPECT_FALSE(observation.reached);
  EXPECT_EQ(observation.reason, "route_phase_pending");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}