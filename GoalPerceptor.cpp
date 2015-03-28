/**
* @file GoalPerceptor.cpp
* @author Michel Bartsch - B-Human Member
* @author Thomas Münder - B-Human Member
* @author Aref Moqadam - MRL-SPL Member
*/

#include "GoalPerceptor.h"
#include <algorithm>

void GoalPerceptor::update(GoalPercept& percept)
{
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Scans", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Validation", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor:removals", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("abol", "drawingOnImage");

  // [FIXME] add body countour to remove the lower tof.
  if (theCameraInfo.camera == CameraInfo::lower) return; // this is a tof for ignoring lower camer. because some part of body would be detected as the goal post.

  //clear old data
  spots.clear();
  percept.goalPosts.clear();

  if(!theCameraMatrix.isValid)
  {
    return;
  }

  int scanHeight = std::max(1, (int)theImageCoordinateSystem.origin.y);
  scanHeight = std::min(scanHeight, theImage.height-2);
  LINE("module:GoalPerceptor:Spots", 1, scanHeight, theImage.width-1, scanHeight, 1, Drawings::ps_dash, ColorClasses::orange);

  // find goal spots
  findSpots(scanHeight);
  scanFieldBoundarySpots(); //-- This added by aref

  // process goal spots
  verticalColorScanDown();
  verticalColorScanUp();
  bottomCorrector(); //-- This added by aref
  calculatePosition(scanHeight);
  validate();
  frameCheck(); //-- This added by aref

  posting(percept);
}

int GoalPerceptor::boundaryStepGenerator(int x)
{
  if (!theFieldBoundary.boundaryInImage.size())
    return 0;

  //-- NOTICE >> do not remove this if section, if you need so, edit the lower 'for' initial (i=begin ===> e<begin+1)
  if (x < theFieldBoundary.boundaryInImage.front().x)
    return theFieldBoundary.boundaryInImage.front().x;

  if (x > theFieldBoundary.boundaryInImage.back().x)
    return theFieldBoundary.boundaryInImage.back().x;

  for (FieldBoundary::InImage::const_iterator i=theFieldBoundary.boundaryInImage.begin();
      i<theFieldBoundary.boundaryInImage.end(); i++)
    if (x < i->x)
      /*
       *  ∆Y     ∆y
       * ―――― = ――――
       *  ∆X     ∆x
       *
       *  Where:
       *   - Y: Vertical distance between Reference and Final (i.y - j.y)
       *   - X: Horizontal distance between Reference and Final (i.x - j.x)
       *   - y: Vertical distance between Reference and Current Point (i.y - output)
       *   - x: Horizontal distance between Reference and Current Point (i.x - x)
       *
       *   Sorry for long comment and complexity below! ;)
       */
      return (i->y-(i-1)->y)*(x-i->x)/(i->x-(i-1)->x)+i->y;

  return 0;
}

void GoalPerceptor::scanFieldBoundarySpots()
{
  Spot candidateSpot(0, 0, 0);
  unsigned char Y=0, Cr=0, Cb=0;
  float g;

  for (int x=0; x<theImage.width-1; x+=2)
  {
    int y=boundaryStepGenerator(x);
    if (y>0 && y<theImage.height && isWhite(x, y))
    {
      Y = theImage[y][x].y;
      Cb = theImage[y][x].cb;
      Cr = theImage[y][x].cr;

      int start, end, noGap=2;
      for (start=y; start>1; start-=2)
        if (isInGrad(x, start, Y, Cr, Cb, g))
          noGap++;
        else if (noGap>1)
          noGap=0;
        else
        {
          start+=2;
          break;
        }

      noGap=2;
      for (end=y; end<theImage.height-1; end+=2)
        if (isInGrad(x, end, Y, Cr, Cb, g))
          noGap++;
        else if (noGap>1)
          noGap=0;
        else
        {
          end+=2;
          break;
        }

      if (end - start < 30)
        continue;

      if (candidateSpot.width == 0)
      {
        candidateSpot.start = x;
        candidateSpot.mid = Vector2<int>(x, y);
        candidateSpot.top = Vector2<int>(candidateSpot.mid.x, start);
        candidateSpot.base = Vector2<int>(candidateSpot.mid.x, end);
      }

      if (candidateSpot.top.y > start)
        candidateSpot.top = Vector2<int>(candidateSpot.mid.x, start);
      if (candidateSpot.base.y < end)
        candidateSpot.base = Vector2<int>(candidateSpot.mid.x, end);

      candidateSpot.end = x+1;
      candidateSpot.width = candidateSpot.end - candidateSpot.start;
    }
    else if (candidateSpot.width < 3)
    {
      candidateSpot = Spot(0, 0, 0);
      Y=Cr=Cb=0;
    }
    else
    {
      RECTANGLE("abol", candidateSpot.start, candidateSpot.top.y, candidateSpot.end, candidateSpot.base.y, 2, Drawings::ps_solid, ColorRGBA(10, 10, 100));
      spots.push_back(candidateSpot);
      candidateSpot = Spot(0, 0, 0);
      Y=Cr=Cb=0;
    }
  }
}

void GoalPerceptor::findSpots(const int& height)
{
  int start;
  int sum;
  int skipped;

  for(int i = 0; i < theImage.width; i++)
  {
    if(isWhite(i, height))
    {
      start = i;
      sum = 0;
      skipped = 0;
      while (i < theImage.width && skipped < yellowSkipping)
      {
        if (isWhite(i, height))
        {
          sum++;
          skipped = 0;
        }
        else
        {
          skipped++;
        }
        i++;
      }

      if(sum > 0) // do not allow posts with width = 0
      {
        spots.push_back(Spot(start, i-skipped, height));
        CROSS("module:GoalPerceptor:Spots", start, height, 2, 2, Drawings::ps_solid, ColorClasses::green);
        CROSS("module:GoalPerceptor:Spots", i-skipped, height, 2, 2, Drawings::ps_solid, ColorClasses::blue);
      }
    }
  }
}

void GoalPerceptor::verticalColorScanDown()
{
  for(std::list<Spot>::iterator i = spots.begin(), end = spots.end(); i != end; ++i)
  {
    Vector2<int> mid = i->mid;
    Vector2<int> lastMid = Vector2<int>(0, 0);
    int width = i->width;
    int baseY = 0;

    while(mid.x != lastMid.x)
    {
      int noGaps = 2;
      for(baseY = mid.y+1; baseY < theImage.height-1; baseY++)
      {
        if(isWhite(mid.x, baseY))
        {
          noGaps++;
        } else if(noGaps > 1)  {
          noGaps = 0;
        } else {
          baseY -= 2;
          break;
        }
      }
      LINE("module:GoalPerceptor:Scans", mid.x, mid.y, mid.x, baseY, 1, Drawings::ps_solid, ColorClasses::yellow);
      lastMid = mid;
      mid.y = mid.y + (baseY-mid.y)/2;
      noGaps = 2;
      int left = 1;
      for(int x = mid.x; x > 1; x--)
      {
        if(isWhite(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          left = x+2;
          break;
        }
      }
      noGaps = 2;
      width = theImage.width - left;
      for(int x = mid.x; x < theImage.width-1; x++)
      {
        if(isWhite(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          width = x-2 - left;
          break;
        }
      }
      i->widths.push_back(width);
      mid.x = left+width/2;
    }
    i->base = Vector2<int>(mid.x, baseY + 1);
    CROSS("module:GoalPerceptor:Scans", i->base.x, i->base.y, 2, 2, Drawings::ps_solid, ColorClasses::red);
  }
}

void GoalPerceptor::verticalColorScanUp()
{
  for(std::list<Spot>::iterator i = spots.begin(), end = spots.end(); i != end; ++i)
  {
    Vector2<int> mid = i->mid;
    Vector2<int> lastMid = Vector2<int>(0, 0);
    int width = i->width;
    int initialWidth = 0;
    int topY = 0;
    int left = i->mid.x-width/2;
    int right = i->mid.x+width/2;
    int lastLeft;
    int lastRight;
    bool crossbarChecking = false;

    while(mid.y != lastMid.y)
    {
      int noGaps = 2;
      for(topY = mid.y-1; topY > 0; topY--)
      {
        if(isWhite(mid.x, topY))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          topY += 2;
          break;
        }
      }
      LINE("module:GoalPerceptor:Scans", mid.x, mid.y, mid.x, topY, 1, Drawings::ps_solid, ColorClasses::yellow);
      lastMid = mid;
      mid.y = mid.y - (mid.y-topY)/2;
      noGaps = 2;
      lastLeft = left;
      lastRight = right;
      left = 1;
      right = theImage.width-1;
      for(int x = mid.x; x > 1; x--)
      {
        if(isWhite(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          left = x+2;
          break;
        }
      }
      noGaps = 2;
      for(int x = mid.x; x < theImage.width-1; x++)
      {
        if(isWhite(x, mid.y))
        {
          noGaps++;
        }
        else if(noGaps > 1)
        {
          noGaps = 0;
        }
        else
        {
          right = x-2;
          width = right-left;
          break;
        }
      }
      if(!initialWidth)
      {
        if(lastLeft > 1 && lastRight < theImage.width-2)
        {
          initialWidth = width;
          crossbarChecking = true;
        }
      }
      if(crossbarChecking)
      {
        if(lastLeft-left > initialWidth && abs(right-lastRight) < initialWidth)
        {
          i->top = Vector2<int>(mid.x, topY);
          i->leftRight = GoalPost::Position::IS_RIGHT;
          LINE("module:GoalPerceptor:Scans", mid.x, mid.y, left, mid.y, 1, Drawings::ps_solid, ColorClasses::yellow);
          goto end_vcs_up;
        }
        else if(right-lastRight > initialWidth && abs(left-lastLeft) < initialWidth)
        {
          i->top = Vector2<int>(mid.x, topY);
          i->leftRight = GoalPost::Position::IS_LEFT;
          LINE("module:GoalPerceptor:Scans", mid.x, mid.y, right, mid.y, 1, Drawings::ps_solid, ColorClasses::yellow);
          goto end_vcs_up;
        }
      }
      mid.x = left+width/2;
    }
    i->leftRight = GoalPost::Position::IS_UNKNOWN;
    end_vcs_up:
    i->top = Vector2<int>(mid.x, topY + 1);
    CROSS("module:GoalPerceptor:Scans", i->top.x, i->top.y, 2, 2, Drawings::ps_solid, ColorClasses::orange);
  }
}

void GoalPerceptor::bottomCorrector()
{
  for (std::list<Spot>::iterator it=spots.begin(); it!=spots.end(); it++)
  {
    int noGaps = 2;
    float grad=0;
    int botY;
    for(botY = it->base.y-5; botY < theImage.height-1; botY++)
    {
      if(isInGrad(it->base.x, botY, grad)) {
        noGaps++;
      } else if(noGaps > 1) {
        noGaps = 0;
      } else {
        botY -= 2;
        break;
      }
    }

    it->base.y = botY;
  }
}

void GoalPerceptor::calculatePosition(const int& height)
{
  for(std::list<Spot>::iterator i = spots.begin(), end = spots.end(); i != end; ++i)
  {
    if (i->base.y > theImage.height-5)
    {
      bool matching = false;
      Vector2<> lastPosition;
      if(theCameraInfo.camera == CameraInfo::upper)
      {
        Vector2<int> projection;
        for(unsigned e = 0; e < lastPosts.size(); e++)
        {
          Vector2<> updated = lastPosts[e].position;
          updated = updated.rotate(-theOdometer.odometryOffset.rotation);
          updated -= theOdometer.odometryOffset.translation;
          Geometry::calculatePointInImage(updated, theCameraMatrix, theCameraInfo, projection);
          if(projection.x < i->end && projection.x > i->start)
          {
            Vector2<int> intersection;
            Geometry::Line l1 = Geometry::Line(Vector2<int>(i->start, height), (Vector2<int>(i->end, height) - Vector2<int>(i->start, height)));
            Geometry::Line l2 = Geometry::Line(projection, (i->base - projection));
            Geometry::getIntersectionOfLines(l1, l2, intersection);
            if(intersection.x < i->end && intersection.x > i->start)
            {
              matching = true;
              lastPosition = updated;
            }
          }
        }
      }
      if(matching)
      {
        i->position = lastPosition;
      }
      else
      {
        float distance = Geometry::getDistanceBySize(theCameraInfo, theFieldDimensions.goalPostRadius * 2.f, (float)i->width);
        Vector2<> angle;
        Geometry::calculateAnglesForPoint(Vector2<>(i->mid), theCameraMatrix, theCameraInfo, angle);
        i->position = Vector2<>(std::cos(angle.x), std::sin(angle.x)) * distance;
      }
    }
    else
    {
      Vector2<> pCorrected = theImageCoordinateSystem.toCorrected(i->base);
      Geometry::calculatePointOnField((int)pCorrected.x, (int)pCorrected.y, theCameraMatrix, theCameraInfo, i->position);
    }
  }
}

// probability based evaluation
void GoalPerceptor::validate()
{
  int distanceEvaluation;
  int relationWidthToHeight;
  int minimalHeight;
  int belowFieldBorder;
  int constantWidth;
  int expectedWidth;
  int expectedHeight;
  int distanceToEachOther;
  int matchingCrossbars;

  int height;
  float value;
  float expectedValue;
  float maxDistance = (Vector2<>(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder) - Vector2<>(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder)).abs() * 1.3f;

  for(std::list<Spot>::iterator i = spots.begin(); i != spots.end(); i++)
  {
    height = (i->base - i->top).abs();

    // if goal post is too far away or too near this post gets 0 %
    i->position.abs() > maxDistance || i->position.abs() < theFieldDimensions.goalPostRadius ? distanceEvaluation = 0 : distanceEvaluation = 1;

    //minimum height
    height < Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, maxDistance) ? minimalHeight = 0 : minimalHeight = 1;

    //if goal post base is above the field border
    value = (float)theFieldBoundary.getBoundaryY(i->base.x);
    i->base.y > value - (value / 20) ? belowFieldBorder = 1 : belowFieldBorder = 0;

    //if all width of the goal posts are alike
    constantWidth = 1;
    for(int w : i->widths){if(w > i->width * 2) constantWidth = 0;}

    //goal posts relation of height to width
    value = ((float)height) / i->width;
    expectedValue = theFieldDimensions.goalHeight / (theFieldDimensions.goalPostRadius * 2);
    relationWidthToHeight = (int)(100 - (std::abs(expectedValue - value) / expectedValue) * 50);

    //distance compared to width
    expectedValue = Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalPostRadius * 2, i->position.abs());
    //clipping with left image limit
    if(i->base.x < (expectedValue / 2))
    {
      expectedValue -= ((expectedValue / 2) - i->base.x);
    }
    //clipping with right image limit
    if(((theImage.width - 1) - i->base.x) < (expectedValue / 2))
    {
      expectedValue -= ((expectedValue / 2) - ((theImage.width - 1) - i->base.x));
    }
    expectedWidth = (int)(100 - (std::abs(expectedValue - i->width) / expectedValue) * 50);

    //distance compared to height
    expectedValue = Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, i->position.abs());
    //clipping with upper image limit
    if(i->base.y < expectedValue)
    {
      expectedValue -= (expectedValue - i->base.y);
    }
    //clipping with lower image limit
    if(((theImage.height - 1) - i->top.y) < expectedValue)
    {
      expectedValue -= (expectedValue - ((theImage.height - 1) - i->top.y));
    }
    expectedHeight = (int)(100 - (std::abs(expectedValue - height) / expectedValue) * 50);

    distanceToEachOther = quality;
    matchingCrossbars = quality;
    if(spots.size() > 1)
    {
      for(std::list<Spot>::iterator j = spots.begin(); j != spots.end(); j++)
      {
        if(i != j)
        {
          //distance to each other
          value = (float)(i->position - j->position).abs();
          expectedValue = std::abs(theFieldDimensions.yPosLeftGoal) * 2;
          if((int)(100 - (std::abs(expectedValue - value) / expectedValue) * 50) > 75) distanceToEachOther = 75;

          //matching crossbars
          if((i->leftRight == GoalPost::IS_LEFT && j->leftRight == GoalPost::IS_RIGHT) || (i->leftRight == GoalPost::IS_RIGHT && j->leftRight == GoalPost::IS_LEFT)) matchingCrossbars = 75;
        }
      }
    }

    if(relationWidthToHeight < 0)
      relationWidthToHeight *= 3;
    if(expectedWidth < 0)
      expectedWidth *= 3;
    if(expectedHeight < 0)
      expectedHeight *= 3;

    i->validity = ((relationWidthToHeight + expectedWidth + expectedHeight + distanceToEachOther + matchingCrossbars) / 5.0f) * distanceEvaluation * minimalHeight * belowFieldBorder * constantWidth;

    int low = 0;
    int high = 110;
    MODIFY("module:GoalPerceptor:low", low);
    MODIFY("module:GoalPerceptor:high", high);

    if(i->validity < high && i->validity > low){
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 55, 10, ColorClasses::black, "distanceEvaluation: " << distanceEvaluation);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 44, 10, ColorClasses::black, "minimalHeight: " << minimalHeight);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 33, 10, ColorClasses::black, "belowFieldBorder: " << belowFieldBorder);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 22, 10, ColorClasses::black, "constantWidth: " << constantWidth);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y - 11, 10, ColorClasses::black, "relationWidthToHeight: " << relationWidthToHeight);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y     , 10, ColorClasses::black, "expectedWidth: " << expectedWidth);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 11, 10, ColorClasses::black, "expectedHeight: " << expectedHeight);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 22, 10, ColorClasses::black, "distanceToEachOther: " << distanceToEachOther);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 33, 10, ColorClasses::black, "matchingCrossbars: " << matchingCrossbars);
      DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, i->mid.y + 44, 10, ColorClasses::black, "validity: " << i->validity);
    }
  }
  spots.sort();
}

void GoalPerceptor::frameCheck()
{
  for (std::list<Spot>::iterator i = spots.begin(); i!=spots.end(); )
  {
    bool shouldBeDeleted = false;

    for (std::list<Spot>::iterator j=i; j!=spots.end(); j++)
    {
      if (i==j)
        continue;


      if (fabs((i->position - j->position).absFloat()) < MERGING_MARGIN &&
          i->position.absFloat() > j->position.absFloat())
      {
        CROSS("module:GoalPerceptor:removals", i->top.x, i->top.y, 5, 5, Drawings::bs_solid, ColorRGBA(10, 10, 100));
        DRAWTEXT("module:GoalPerceptor:removals", i->top.x+10, -1 * i->top.y, 7, ColorRGBA(10, 100, 10), fabs((i->position - j->position).absFloat()));
        shouldBeDeleted = true;
        break;
      }
    }

    if (shouldBeDeleted)
      i=spots.erase(i);
    else
      i++;
  }
}

void GoalPerceptor::posting(GoalPercept& percept)
{
  lastPosts.clear();
  if(!spots.empty())
  {
    Spot first = spots.back();
    if(first.validity > quality)
    {
      GoalPost p1;
      p1.position = first.leftRight;
      p1.positionInImage = first.base;
      p1.positionOnField = first.position;
      spots.pop_back();
      if(!spots.empty())
      {
        Spot second = spots.back();
        if(second.validity > quality)
        {
          GoalPost p2;
          p2.position = second.leftRight;
          p2.positionInImage = second.base;
          p2.positionOnField = second.position;

          if(p1.positionInImage.x < p2.positionInImage.x)
          {
            p1.position = GoalPost::Position::IS_LEFT;
            p2.position = GoalPost::Position::IS_RIGHT;
          }
          else
          {
            p1.position = GoalPost::Position::IS_RIGHT;
            p2.position = GoalPost::Position::IS_LEFT;
          }
          percept.goalPosts.push_back(p2);
          percept.timeWhenCompleteGoalLastSeen = theFrameInfo.time;
          if(theCameraInfo.camera == CameraInfo::lower)
        	  lastPosts.push_back(second);
        }
      }
      percept.goalPosts.push_back(p1);
      percept.timeWhenGoalPostLastSeen = theFrameInfo.time;
      if(theCameraInfo.camera == CameraInfo::lower)
    	  lastPosts.push_back(first);
    }
  }
}

inline bool GoalPerceptor::isWhite(const int& x, const int& y)
{
  return theColorReference.isWhite(&theImage[y][x]);
}

inline bool GoalPerceptor::isInGrad(int px, int py, unsigned char& Y, unsigned char& Cr, unsigned char& Cb, float& diff2)
{
  if (!theColorReference.isWhite(&theImage[py][px]))
    return false;

  const float y  = theImage[py][px].y;
  const float cr = theImage[py][px].cr;
  const float cb = theImage[py][px].cb;

  diff2 = (y-Y)*(y-Y) + (cr-Cr)*(cr-Cr) + (cb-Cb)*(cb-Cb);

  Y = y;
  Cr = cr;
  Cb = cb;

  return (diff2 < COLOR_DIFFRENCE_VALUE);
}

inline bool GoalPerceptor::isInGrad(const int& px, const int& py, float& grad)
{
  const unsigned char y  = theImage[py][px].y;
  const unsigned char cb = theImage[py][px].cb;
  const unsigned char cr = theImage[py][px].cr;

  return (y>110 && abs(cb-127)<50 && abs(cr-127)<50);
//  return !theColorReference.isGreen(&theImage[y][x]);
}

MAKE_MODULE(GoalPerceptor, Perception)
