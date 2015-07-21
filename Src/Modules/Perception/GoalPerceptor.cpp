/**
 * @file GoalPerceptor.cpp
 * @author <a href="mailto:a.moqadammehr@mrl-spl.ir">Aref Moqadam</a> - MRL-SPL Member
 * @author Michel Bartsch - B-Human Member
 * @author Thomas Münder - B-Human Member
 */

#include "GoalPerceptor.h"
#include "Platform/Common/File.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string.h>

GoalPerceptor::GoalPerceptor() :
	candidateSpot(0 , 0 , 0 ) ,
	RobotRejection(false)
{
}

void GoalPerceptor::update(GoalPercept& percept)
{
	RobotRejection = false;
	DEBUG_RESPONSE("module:GoalPerceptor:rejectRobots", RobotRejection = true; );

	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Spots", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Scans", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Validation", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:removals", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:Candidates", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:MidPoints", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:ShapeScans", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:GoalPerceptor:LowerPoint", "drawingOnImage");

	MODIFY("module:GoalPerceptor:minVotePoint", minVotePoint);
	MODIFY("module:GoalPerceptor:quality", quality);
	MODIFY("module:GoalPerceptor:colorDifference", colorDifferenceValue);

	//-- clear old data
	percept.goalPosts.clear();
	spots.clear();

	if(!theCameraMatrix.isValid)
		return;

	//-- Scan height is equaling with horizon clipped by image boundaries.
	int scanHeight = std::max(1, (int)theImageCoordinateSystem.origin.y);
	scanHeight = std::min(scanHeight, theImage.height-2);
	LINE("module:GoalPerceptor:Spots", 1, scanHeight, theImage.width-1, scanHeight, 1, Drawings::ps_dash, ColorClasses::orange);

	//-- Find the possible goal-posts
	scanFieldBoundarySpots(scanHeight);

	//-- Process possibilities
	verticalColorScanDown();
	verticalColorScanUp();
	clipSpotBoundaries();

	//-- Validation checks
	if (RobotRejection)
	  rejectRobot();
	calculateVotePoints(); //-- This function just remove the spots with low vote point percentage
	// bottomCorrector(); //-- Commented in RC2015
	calculatePosition(scanHeight);
	validate();
	removeNotGoalposts();

	//-- Export the results
	posting(percept);
}

void GoalPerceptor::clipSpotBoundaries()
{
  for (Spot& s : spots)
  {
    RECTANGLE("module:GoalPerceptor:ShapeScans", s.top.x, s.top.y, s.base.x, s.base.y, 4, Drawings::ps_solid, ColorClasses::red);

    if (s.base.x < s.start)
      s.base.x = s.start;
    if (s.top.x > s.end)
      s.top.x = s.end;

    RECTANGLE("module:GoalPerceptor:ShapeScans", s.top.x, s.top.y, s.base.x, s.base.y, 2, Drawings::ps_dot, ColorClasses::yellow);
  }
}

int GoalPerceptor::boundaryStepGenerator(int x)
{
	if (!theFieldBoundary.boundaryInImage.size())
		throw("no boundary in the image");

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

	return -1;
}

void GoalPerceptor::scanFieldBoundarySpots(const int& height)
{
	unsigned char Y=0, Cr=0, Cb=0;

	int noGapX = 2;
	for (int x=0; x<theImage.width-1; x+=2)
	{
		int y=boundaryStepGenerator(x);
		if (y>-1 && y<theImage.height && isWhite(x, y))
		{
			noGapX++;
			Y = theImage[y][x].y;
			Cb = theImage[y][x].cb;
			Cr = theImage[y][x].cr;

			int start, end, noGap=2;
			for (start=y; start>1; start-=2)
				if (isInGrad(x, start, Y, Cr, Cb))
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
				if (isInGrad(x, end, Y, Cr, Cb))
					noGap++;
				else if (noGap>1)
					noGap=0;
				else
				{
					end-=2;
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
		else if (noGapX > 1)
		{
			noGapX = 0;
		}
		else if (candidateSpot.width < 3)
		{
			candidateSpot = Spot(0, 0, 0);
			Y=Cr=Cb=0;
		}
		else
		{
			RECTANGLE("module:GoalPerceptor:Candidates", candidateSpot.start, candidateSpot.top.y, candidateSpot.end, candidateSpot.base.y, 2, Drawings::ps_solid, ColorRGBA(10, 10, 100));
			candidateSpot.mid.x = (candidateSpot.start+candidateSpot.end)/2;

			if (candidateSpot.top.y <= height)
				spots.push_back(candidateSpot);

			candidateSpot = Spot(0, 0, 0);
			Y=Cr=Cb=0;
		}
	}
}


void GoalPerceptor::findSpots(const int& height)
{
  // [XXX] : unused function
  ASSERT(false);
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
	  int totalPoints = 0;
	  int positivePoints = 0;

		Vector2<int> mid = i->mid;
		Vector2<int> lastMid = Vector2<int>(0, 0);
		int width = i->width;
		int baseY = 0;

		while(mid.x != lastMid.x && i->start < mid.x && mid.x < i->end)
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

			//-- Calculate vote point
			for (int vc=0; vc<15; vc+=3)
			{
			  DOT("module:GoalPerceptor:LowerPoint", mid.x, baseY+vc, ColorClasses::blue, ColorClasses::blue);
			  totalPoints++;
			  if (theColorReference.isGreen(theImage[baseY+vc]+mid.x))
			    positivePoints+=100;
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
		i->votePoint = positivePoints / totalPoints;
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
		int botY;
		for(botY = it->base.y-5; botY < theImage.height-1; botY++)
		{
			if(isInGrad(it->base.x, botY, 10)) {
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
			Vector2<> pCorrected = theImageCoordinateSystem.toCorrected(Vector2<int>((i->start + i->end)/2.f, i->base.y));
			Geometry::calculatePointOnField((int)pCorrected.x, (int)pCorrected.y, theCameraMatrix, theCameraInfo, i->position);
		}
	}
}

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

		// minimum height
		height < Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, maxDistance) ? minimalHeight = 0 : minimalHeight = 1;

		// if goal post base is above the field border
		value = (float)theFieldBoundary.getBoundaryY(i->base.x);
		i->base.y > value - (value / 20) ? belowFieldBorder = 1 : belowFieldBorder = 0;

		// if all width of the goal posts are alike
		constantWidth = 1;
		// [FIXME] : Since the goal posts are not white any more, the edge detection
		//           is not working same as it was before. Hence, the width validation
		//           is not used.
		//    for(int w : i->widths){if(w > i->width * 2) constantWidth = 0;}

		// goal posts relation of height to width
		value = ((float)height) / i->width;
		expectedValue = theFieldDimensions.goalHeight / (theFieldDimensions.goalPostRadius * 2);
		relationWidthToHeight = (int)(100 - (std::abs(expectedValue - value) / expectedValue) * 50);

		// distance compared to width
		expectedValue = Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalPostRadius * 2, i->position.abs());
		
		// clipping with left image limit
		if(i->base.x < (expectedValue / 2))
			expectedValue -= ((expectedValue / 2) - i->base.x);

		// clipping with right image limit
		if(((theImage.width - 1) - i->base.x) < (expectedValue / 2))
			expectedValue -= ((expectedValue / 2) - ((theImage.width - 1) - i->base.x));
		expectedWidth = (int)(100 - (std::abs(expectedValue - i->width) / expectedValue) * 50);

		// distance compared to height
		expectedValue = Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, i->position.abs());
		
		// clipping with upper image limit
		if(i->base.y < expectedValue)
			expectedValue -= (expectedValue - i->base.y);
			
		// clipping with lower image limit
		if(((theImage.height - 1) - i->top.y) < expectedValue)
			expectedValue -= (expectedValue - ((theImage.height - 1) - i->top.y));
		expectedHeight = (int)(100 - (std::abs(expectedValue - height) / expectedValue) * 50);

		distanceToEachOther = quality;
		matchingCrossbars = quality;
		if(spots.size() > 1)
		{
			for(std::list<Spot>::iterator j = spots.begin(); j != spots.end(); j++)
			{
				if(i != j)
				{
					// distance to each other
					value = (float)(i->position - j->position).abs();
					expectedValue = std::abs(theFieldDimensions.yPosLeftGoal) * 2;
					if((int)(100 - (std::abs(expectedValue - value) / expectedValue) * 50) > 75) distanceToEachOther = 75;

					// matching crossbars
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

		i->validity = ((relationWidthToHeight +
		                expectedWidth +
		                /*expectedHeight + */ // [FIXME] : This is commented because our head control engine
		                                      //           is always looks down, though it can not see top of 
		                                      //           the goal posts. So, it does not make any sence...
		                distanceToEachOther +
		                matchingCrossbars) / 5.0f) *
		                
		                distanceEvaluation *
		                minimalHeight *
		                belowFieldBorder *
		                constantWidth;

		int low = 0;
		int high = 110;
		MODIFY("module:GoalPerceptor:low", low);
		MODIFY("module:GoalPerceptor:high", high);

		//-- Debugging:
		{
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y - 55, 10, ColorClasses::black, "distanceEvaluation: " << distanceEvaluation);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y - 44, 10, ColorClasses::black, "minimalHeight: " << minimalHeight);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y - 33, 10, ColorClasses::black, "belowFieldBorder: " << belowFieldBorder);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y - 22, 10, ColorClasses::black, "constantWidth: " << constantWidth);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y - 11, 10, ColorClasses::black, "relationWidthToHeight: " << relationWidthToHeight);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y     , 10, ColorClasses::black, "expectedWidth: " << expectedWidth);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y + 11, 10, ColorClasses::black, "expectedHeight: " << expectedHeight);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y + 22, 10, ColorClasses::black, "distanceToEachOther: " << distanceToEachOther);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y + 33, 10, ColorClasses::black, "matchingCrossbars: " << matchingCrossbars);
			DRAWTEXT("module:GoalPerceptor:Validation", i->mid.x, -i->mid.y + 44, 10, ColorClasses::black, "validity: " << i->validity);
		}
	}
	
	spots.sort();
}

void GoalPerceptor::removeNotGoalposts()
{
  for (std::list<Spot>::iterator i = spots.begin(); i!=spots.end(); )
  {
    bool shouldBeDeleted = false;

    //-- Check for body contour
    int hackForBodyContourFunction = i->base.y;
    theBodyContour.clipBottom(i->base.x, hackForBodyContourFunction);
    if (i->base.y != hackForBodyContourFunction)
    {
      CROSS("module:GoalPerceptor:removals", i->base.x, i->base.y, 5, 5, Drawings::bs_solid, ColorRGBA(100, 10, 10)); //-- Dark Red
      shouldBeDeleted = true;
    }

    //-- Check for duplications
    for (std::list<Spot>::iterator j=i; !shouldBeDeleted && j!=spots.end(); j++)
    {
      if (i==j)
        continue;

      if (abs(i->mid.x - j->mid.x) < 5)
      {
        CROSS("module:GoalPerceptor:removals", i->mid.x, i->mid.y, 5, 5, Drawings::bs_solid, ColorRGBA(200, 10, 10)); //-- Light Red
        shouldBeDeleted = true;
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
	COMPLEX_DRAWING("module:GoalPerceptor:MidPoints", {
			for (const Spot& s : spots)
				CROSS("module:GoalPerceptor:MidPoints", s.mid.x, s.mid.y, 3, 3, Drawings::ps_solid, ColorClasses::orange);
	});

	lastPosts.clear();
	if(!spots.empty())
	{
		Spot first = spots.back();
		if(first.validity > quality)
		{
			GoalPost p1;
			p1.position = first.leftRight;
			p1.positionInImage = Vector2<int>((first.start + first.end)/2.f, first.base.y);
			p1.positionOnField = first.position;
			spots.pop_back();
			if(!spots.empty())
			{
				Spot second = spots.back();
				if(second.validity > quality)
				{
					GoalPost p2;
					p2.position = second.leftRight;
					p2.positionInImage = Vector2<int>((second.start + second.end)/2.f, second.base.y);
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

void GoalPerceptor::calculateVotePoints()
{
  for (std::list<Spot>::iterator i=spots.begin(); i!=spots.end();)
  {
    //-- Removing noise from the list
    if (i->votePoint < minVotePoint)
    {
      CROSS("module:GoalPerceptor:removals", i->base.x, i->base.y, 3, 3, Drawings::bs_solid, ColorClasses::green);
      DRAWTEXT("module:GoalPerceptor:removals", i->base.x, -i->base.y + 7, 5, ColorClasses::green, i->votePoint);
      i = spots.erase(i);
    }
    else
      i++;
  }
}

void GoalPerceptor::rejectRobot()
{
	for (std::list<Spot>::iterator i=spots.begin(); i!=spots.end(); )
	{
		bool shouldBeDeleted = false;
		for (auto& r : theRobotPercept.robots)
		{
			if (r.detectedJersey && r.x1 < i->mid.x && i->mid.x < r.x2)
			{
				shouldBeDeleted = true;
				break;
			}
		}

		if (shouldBeDeleted)
		{
		  CROSS("module:GoalPerceptor:removals", i->base.x, i->base.y, 3, 3, Drawings::bs_solid, ColorRGBA(10, 10, 120)); //-- Blue
			i = spots.erase(i);
		}
		else
			i++;
	}
}

inline bool GoalPerceptor::isWhite(const int& x, const int& y)
{
	return theColorReference.isYellow(&theImage[y][x]);
}

inline bool GoalPerceptor::isInGrad(int px, int py, unsigned char& Y, unsigned char& Cr, unsigned char& Cb)
{
	if (!theColorReference.isYellow(&theImage[py][px]))
		return false;

	const float y  = theImage[py][px].y;
	const float cr = theImage[py][px].cr;
	const float cb = theImage[py][px].cb;

	const float diff2 = (y-Y)*(y-Y) + (y-Y)*(y-Y) + (cr-Cr)*(cr-Cr) + (cb-Cb)*(cb-Cb);

	Y = y;
	Cr = cr;
	Cb = cb;

	return (diff2 < colorDifferenceValue);
}

MAKE_MODULE(GoalPerceptor, Perception)

