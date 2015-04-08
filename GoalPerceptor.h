/**
* @file GoalPerceptor.cpp
* @author <a href="mailto:a.moqadammehr@mrl-spl.ir">Aref Moqadam</a> - MRL-SPL Member
* @author Michel Bartsch - B-Human Member
* @author Thomas Münder - B-Human Member
*/
#pragma once

#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Perception/ColorReference.h"

MODULE(GoalPerceptor)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(Image)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(ColorReference)
  REQUIRES(FieldBoundary)
  REQUIRES(Odometer)
  PROVIDES_WITH_MODIFY_AND_DRAW(GoalPercept)
  LOADS_PARAMETER(int, quality)
  LOADS_PARAMETER(int, yellowSkipping)
  LOADS_PARAMETER(int, mergingMargin)
  LOADS_PARAMETER(int, colorDifferenceValue)
END_MODULE

/**
 * @class GoalPerceptor
 */
class GoalPerceptor: public GoalPerceptorBase
{
private:
  struct Spot
  {
  public:
    Spot(int s, int e, int h) : start(s), end(e), validity(100), leftRight(GoalPost::IS_UNKNOWN)
    {
      width = end - start;
      mid = Vector2<int>(start+(width / 2), h);
    }

    inline bool operator<(const Spot& other)const
    {
      return validity < other.validity;
    }

    int start;
    int end;
    int width;
    std::vector<int> widths;
    Vector2<int> mid;
    Vector2<int> base;
    Vector2<int> top;
    GoalPost::Position leftRight;
    Vector2<> position;
    float validity;
  };

  // [TODO] : find a better class to use instead of this one. Or at least move it to somewhere else.
  class Point
  {
  public:
    Point(int X=0, int Y=0) : x(X), y(Y) {}
    int x, y;

    bool operator < (const Point& other) const { return this->x < other.x; }
  };

  void update(GoalPercept& percept);

  void findSpots(const int& height);

  void verticalColorScanDown();

  void verticalColorScanUp();

  bool isWhite(const int& x, const int& y);

  bool isInGrad(const int& x, const int& y);

  bool isInGrad(int x, int y, unsigned char& Y, unsigned char& Cr, unsigned char& Cb);

  void calculatePosition(const int& height);

  std::vector<Point> trimData(const std::vector<Point>& input);

  float PearsonCorrelation(const std::vector<Point>& input);

  void validate();

  void posting(GoalPercept& percept);

  void frameCheck();

  void bottomCorrector();

  void checkVerticalShape();

  int boundaryStepGenerator(int x);

  void scanFieldBoundarySpots(const int& height);

  std::list<Spot> spots;

  std::vector<Spot> lastPosts;
};
