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
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/BodyContour.h"

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
  REQUIRES(RobotPercept)
  REQUIRES(BodyContour)
  PROVIDES_WITH_MODIFY_AND_DRAW(GoalPercept)
  LOADS_PARAMETER(int, quality)
  LOADS_PARAMETER(int, yellowSkipping)
  LOADS_PARAMETER(int, colorDifferenceValue)
  LOADS_PARAMETER(float, minVotePoint)
END_MODULE

/**
 * @class GoalPerceptor
 */
class GoalPerceptor: public GoalPerceptorBase
{
public:
  /**
   * @brief Default constructor for goalPerceptor class
   */
  GoalPerceptor();

private:
  /**
   * @class Spot
   * @brief Contains information about a possible spot for goal post
   */
  struct Spot
  {
  public:
    Spot(int s, int e, int h) : start(s), end(e), validity(100), votePoint(0), leftRight(GoalPost::IS_UNKNOWN)
    {
      width = end - start;
      mid = Vector2<int>(start+(width / 2), h);
    }

    inline bool operator<(const Spot& other)const
    {
      return validity < other.validity;
    }

    int start; /// Starting margin of the spot in horizontal axie
    int end; /// Final margin of the spot in horizontal axie
    int width; /// Width of margin (final edge - starting edge)
    float votePoint; /// The score that the spot reached by scanning its lower boundary
    std::vector<int> widths; /// Width of the scanned lines
    Vector2<int> mid; /// The middle point in horizontal axie
    Vector2<int> base; /// The lowest point (top-left duo to image coordination) of the spot
    Vector2<int> top; /// The highest point (bottom-right duo to image coordination) of the spot
    GoalPost::Position leftRight; /// Enumeration to demonstrate whearas the post is right one or left one.
    Vector2<> position; /// Extracted position of the spot in image
    float validity; /// Score the spot has reached by defiend check points
  };

  // [TODO] : This class should not be here, hence I rather not no doxygen it...
  class Point
  {
  public:
    Point(int X=0, int Y=0) : x(X), y(Y) {}
    int x, y;

    bool operator < (const Point& other) const { return this->x < other.x; }
  };


  /**
   * @brief The main function that detects the goal posts and updates the informations.
   * @param percept: Pointer to the object to be update
   */
  void update(GoalPercept& percept);

  /**
   * @brief Check the given height for white spots
   * @param height: Scan height
   */
  void findSpots(const int& height);

  /**
   * @brief Scan down to find the lowest white point for each spot.
   */
  void verticalColorScanDown();

  /**
   * @brief Scan upward to find the highest white point for each spot.
   */
  void verticalColorScanUp();

  /**
   * @brief Check the pixel in the color table to see if it is white (yellow in the CT).
   * @param X, Y: position of the pixel in the image
   * @return True if the color is white
   */
  bool isWhite(const int& x, const int& y);

  /**
   * @brief Track the gradient of the pixels
   * @param x, y: position of the pixel in the image
   * @param Y, Cr, Cb: color of the previous pixel (or any other pixel) that needs to be track
   * @return True if the difference is not quite much
   * @note The difference is in goal perceptor configuration file as 'color difference value'
   */
  bool isInGrad(int x, int y, unsigned char& Y, unsigned char& Cr, unsigned char& Cb);

  /**
   * @brief Calculate the projected position of the goal post on the field.
   */
  void calculatePosition(const int& height);


  /**
   * @brief Validate the spots and rate them by for defined qualifiers.
   */
  void validate();

  /**
   * @brief Select the two best goal post that are qualified.
   */
  void posting(GoalPercept& percept);


  /**
   * @brief Remove the goal-posts that does not satisfy the defined check-list.
   */
  void removeNotGoalposts();

  /**
   * @brief Gives the height of the convex boundary of the given x
   * @param x : position of the required boundary step
   * @return height of the boundary step
   * @throw Exception if there is no convex boundary point
   */
  int boundaryStepGenerator(int x);

  /**
   * @brief Scans the field boundary for any white pixel violation
   * @param height : clipped horizon
   */
  void scanFieldBoundarySpots(const int& height);

  /**
   * @brief Clip the boundary of goal-posts to their candidate limitations
   */
  void clipSpotBoundaries();

  /**
   * @brief Reject spots inside the detected obstacle with jersey
   */
  void rejectRobot();

  /**
   * @brief Remove the spots with low vote point
   */
  void calculateVotePoints();

  /**
   * @brief Percise the lower boundary of the goal posts
   */
  void bottomCorrector();

  Spot candidateSpot;  /// Candidate Iterator on spots
  std::list<Spot> spots; /// Set of candidate spots to be goal post
  std::vector<Spot> lastPosts; /// Goal posts from last farme
  bool RobotRejection; /// Flag to use robot rejection sub-module
};

