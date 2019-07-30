#ifndef ROADOPTIONS_H
#define ROADOPTIONS_H

/*
 * Static road options
 */
struct RoadOptions {

  double lane_width;
  double lane_number;
  double speed_limit;
  double max_s;
  double acc_limit;


  int getLaneNumber(double d) const
  {
     return static_cast<int>(d / lane_width);
  }

  double getLaneCenter(int lane_num) const
  {
      return lane_num * lane_width + lane_width / 2.0;
  }
};


#endif // ROADOPTIONS_H
