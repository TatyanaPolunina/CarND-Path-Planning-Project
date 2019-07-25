#ifndef ROADOPTIONS_H
#define ROADOPTIONS_H


struct RoadOptions {

  double lane_width;
  double lane_number;
  double speed_limit;


  double getLaneNumber(double d) const
  {
     return static_cast<int>(d / lane_width);
  }

  double getLaneCenter(int lane_num) const
  {
      return lane_number * lane_width + lane_width / 2.0;
  }
};


#endif // ROADOPTIONS_H
