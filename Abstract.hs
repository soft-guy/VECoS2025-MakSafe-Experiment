{-# LANGUAGE FunctionalDependencies #-}
{-# LANGUAGE OverloadedStrings #-}

module Abstract where

import Control.Arrow
import Control.Monad (guard)
import Data.Maybe (catMaybes, fromJust)
import Data.Text (Text)
import Data.Text qualified as T
import Data.Text.Lazy.Builder (fromText)
import Data.Tree
import Data.Typeable (Typeable, cast)
import Fmt
import Fmt.Internal.Core (FromBuilder)

-- | 'LaneDirection' represents the driving direction of a lane.
data LaneDirection = PositiveDirection | NegativeDirection
  deriving (Eq, Show, Enum, Bounded, Read, Ord)

-- | Type @lane@ is an instance of class 'Lane' only if its values are
-- lane-like.
class (Eq lane, Ord lane) => Lane lane where
  roadId :: lane -> Int
  longitudinalStart :: lane -> Double
  longitudinalLength :: lane -> Double
  lateralOffset :: lane -> Int
  direction :: lane -> LaneDirection
  longitudinalEnd :: lane -> Double
  longitudinalEnd lane = longitudinalStart lane + longitudinalLength lane
  longitudinalLength lane = longitudinalEnd lane - longitudinalStart lane
  {-# MINIMAL
    roadId,
    longitudinalStart,
    lateralOffset,
    direction,
    (longitudinalLength | longitudinalEnd)
    #-}

-- | The 'laneReport' function generates a textual representation of a lane.
laneReport :: (FromBuilder b, Lane lane) => lane -> b
laneReport lane =
  fmt $
    nameF "Lane" $
      blockListF'
        "@"
        id
        [ "Road: " +| roadId lane |+ "" :: Builder,
          "Longitudinal start: " +| longitudinalStart lane |+ "",
          "Longitudinal end: " +| longitudinalEnd lane |+ "",
          "Lateral offset: " +| lateralOffset lane |+ "",
          "Driving direction: " +|| direction lane ||+ ""
        ]

laneReport' :: (FromBuilder b, Lane lane) => lane -> b
laneReport' lane =
  fmt $
    "Lane that longitudinally spans the interval "
      +| listF (map ($ lane) [longitudinalStart, longitudinalEnd])
      |+ " of road '"
      +| roadId lane
      |+ "' and is the '"
      +| abs (lateralOffset lane)
      |+ "' lane to the "
      +| ( if lateralOffset lane < 0
             then "right"
             else "left"
         )
      +| ";"

-- | 'Position' @lane@ represents a position within @lane@ if @lane@ is an
-- instance of class 'Lane'.
data Position lane where Pos :: (Lane lane) => lane -> Double -> Position lane

instance (Lane lane, Eq lane) => Eq (Position lane) where
  (==) :: (Lane lane, Eq lane) => Position lane -> Position lane -> Bool
  (Pos l1 v1) == (Pos l2 v2) = l1 == l2 && v1 == v2

deriving instance (Lane lane, Ord lane) => Ord (Position lane)

-- | 'mkPosition' accepts a @lane@ along with a 'Double' value and returns a
-- | value of 'Position' @lane@ only if the value is within the range specified
-- | by the @lane@.
mkPosition :: (Lane lane) => lane -> Double -> Maybe (Position lane)
mkPosition l v = do
  guard (longitudinalStart l <= v && v <= longitudinalEnd l)
  return (Pos l v)

-- | 'positionReport' function generates a textual representation of a
-- 'Position' value.
positionReport :: (FromBuilder b) => Position lane -> b
positionReport (Pos l v) =
  fmt $
    nameF "Position" $
      blockListF
        [ "Road: " +| roadId l |+ "" :: Builder,
          "Lateral offset: " +| lateralOffset l |+ "",
          "Longitudinal coordinate: " +| fixedF 2 v |+ ""
        ]

positionReport' :: (FromBuilder b) => Position lane -> b
positionReport' (Pos l v) =
  fmt $
    "road '"
      +| roadId l
      |+ "'; the '"
      +| abs (lateralOffset l)
      |+ "' lane to the "
      +| ( if lateralOffset l < 0
             then "right"
             else "left"
         )
      +| "; longitudinal coordinate "
      +| fixedF 2 v
      |+ ""

-- | 'LaneSegment' @lane@ represents, given a lane, the lane's segment
-- specified by two values interpreted as the segment's start and end.
data LaneSegment lane where
  LnSgmnt :: (Lane lane) => lane -> Double -> Double -> LaneSegment lane

getStartLnSgmnt :: LaneSegment lane -> Double
getStartLnSgmnt (LnSgmnt _ s _) = s

getEndLnSgmnt :: LaneSegment lane -> Double
getEndLnSgmnt (LnSgmnt _ _ e) = e

deriving instance (Eq lane) => Eq (LaneSegment lane)

-- | Given a lane and two 'Double' values, 'mkLnSgmnt' returns a 'LaneSegment' -- only if the given values form a sub-interval within the lane.
mkLnSgmnt :: (Lane lane) => lane -> Double -> Double -> Maybe (LaneSegment lane)
mkLnSgmnt l v1 v2 = do
  let sStart = min v1 v2
  let sEnd = max v1 v2
  guard (longitudinalStart l <= sStart && sEnd <= longitudinalEnd l)
  return $ LnSgmnt l sStart sEnd

-- | 'lnSgmntReport' function generates a textual representation of a
-- 'LaneSegment' value.
lnSgmntReport :: (FromBuilder b, Lane lane) => LaneSegment lane -> b
lnSgmntReport (LnSgmnt l v1 v2) =
  fmt $
    nameF "LaneSegment" $
      blockListF
        [ "Road: " +| roadId l |+ "" :: Builder,
          "Lateral offset: " +| lateralOffset l |+ "",
          "Segment start: " +| v1 |+ "",
          "Segment end: " +| v2 |+ ""
        ]

-- | This function checks whether one can merge two 'LaneSegment' values.
canFuseLnSgmnt2 :: (Eq lane) => LaneSegment lane -> LaneSegment lane -> Bool
canFuseLnSgmnt2 (LnSgmnt l1 s1 e1) (LnSgmnt l2 s2 e2) =
  l1 == l2 && max s1 s2 <= min e1 e2

-- | Given a list of 'LaneSegment' values parametrized by @lane@—an instance
-- of class 'Lane'—one can use 'fuse' to make them fuse and thus receive an
-- equivalent but minimal list of lanes.
fuse :: (Eq lane) => [LaneSegment lane] -> [LaneSegment lane]
fuse = foldr merge []
  where
    merge lnSgmnt [] = [lnSgmnt]
    merge lnSgmnt (l : ls)
      | canFuseLnSgmnt2 lnSgmnt l = merge (fuseTwo lnSgmnt l) ls
      | otherwise = l : merge lnSgmnt ls

    fuseTwo (LnSgmnt l s1 e1) (LnSgmnt _ s2 e2) =
      LnSgmnt l (min s1 s2) (max e1 e2)

-- | This function checks whether two lists of 'LaneSegment' values intersect.
doLaneSegmentsIntersect ::
  (Eq lane) => [LaneSegment lane] -> [LaneSegment lane] -> Bool
doLaneSegmentsIntersect xs ys = or $ liftA2 canFuseLnSgmnt2 xs ys

-- | 'laneSegmentIntersection' returns the intersection of two 'LaneSegment's
-- if they intersect, and 'Nothing' otherwise.
laneSegmentIntersection2 ::
  LaneSegment lane ->
  LaneSegment lane ->
  Maybe (LaneSegment lane)
laneSegmentIntersection2 ls1@(LnSgmnt l1 s1 e1) ls2@(LnSgmnt l2 s2 e2)
  | canFuseLnSgmnt2 ls1 ls2 =
      Just $ LnSgmnt l1 (max s1 s2) (min e1 e2)
  | otherwise = Nothing

-- | 'LaneSegmentOverlap' @lane@ represents that two 'LaneSegment's overlap. It
-- specifies that the lane segments overlap from the physical road network's
-- perspective. The lane segments do not necessarily intersect when overlapping.
data LaneSegmentOverlap lane where
  LaneSegmentOverlap ::
    (Lane lane) =>
    LaneSegment lane ->
    LaneSegment lane ->
    LaneSegmentOverlap lane

-- | A 'Zone' is a collection of 'LaneSegment's.
type Zone lane = [LaneSegment lane]

-- | 'doOverlapLnSgmnt2' checks whether two 'LaneSegment's overlap given a list
-- of 'LaneSegmentOverlap's.
doOverlapLnSgmnt2 ::
  (Eq lane) =>
  [LaneSegmentOverlap lane] ->
  LaneSegment lane ->
  LaneSegment lane ->
  Bool
doOverlapLnSgmnt2 overlaps ls1 ls2 =
  any
    ( \(LaneSegmentOverlap o1 o2) ->
        (canFuseLnSgmnt2 ls1 o1 && canFuseLnSgmnt2 ls2 o2)
          || (canFuseLnSgmnt2 ls1 o2 && canFuseLnSgmnt2 ls2 o1)
    )
    overlaps

-- | 'doZonesOverlap' checks whether two 'Zone's overlap given a list of
-- 'LaneSegmentOverlap's. It returns 'True' if any pair of lane segments from
-- the two zones overlap, and 'False' otherwise.
doZonesOverlap ::
  (Eq lane) =>
  [LaneSegmentOverlap lane] ->
  Zone lane ->
  Zone lane ->
  Bool
doZonesOverlap overlaps zone1 zone2 = or $ liftA2 doLnSgmntsOverlap zone1 zone2
  where
    doLnSgmntsOverlap ls1 ls2 =
      uncurry (||) $
        ((doOverlapLnSgmnt2 overlaps &&& canFuseLnSgmnt2) >>> uncurry (&&&)) ls1 ls2

zoneUnion :: (Eq lane) => Zone lane -> Zone lane -> Zone lane
zoneUnion zone1 zone2 = fuse $ zone1 ++ zone2

laneZone :: (Lane lane) => lane -> Zone lane
laneZone l = return $ LnSgmnt l (longitudinalStart l) (longitudinalEnd l)

zoneIntersection ::
  (Eq lane) => [LaneSegment lane] -> [LaneSegment lane] -> [LaneSegment lane]
zoneIntersection zone1 zone2
  | doLaneSegmentsIntersect zone1 zone2 =
      fuse $ catMaybes $ liftA2 laneSegmentIntersection2 zone1 zone2
  | otherwise = []

-- | 'zoneReport' function generates a textual representation of a 'Zone'.
zoneReport ::
  (FromBuilder b, Lane lane) => Zone lane -> (LaneSegment lane -> Builder) -> b
zoneReport zone laneSegmentBuilder =
  fmt $
    nameF "Zone" $
      blockListF' "*" laneSegmentBuilder zone

-- | This class allows one to express the ability to propagate a position
-- parametrized by type @lane@ in the direction given by a @directionDomain@
-- value as far as specified by a @metricCodomain@ value using values of type
-- @link@.
class
  (Lane lane, Eq lane, Enum directionDomain, Bounded directionDomain) =>
  ProvidesNeighborhoodOfPosition link lane metricCodomain directionDomain
    | link -> lane,
      link -> metricCodomain,
      link -> directionDomain
  where
  neighborhoodOfPosition ::
    Position lane ->
    [link] ->
    metricCodomain ->
    directionDomain ->
    [LaneSegment lane]
  -- ^ Given a specified position, the connectivity links, the designated
  -- propagation distance, and the corresponding direction, the function shall
  -- return lane segments that collectively delineate some neighborhood
  -- surrounding that position.

  -- | This function allows one to propagate a given position via specified
  -- links by a designated distance in all directions, yielding the aggregated
  -- neighborhood.
  ballCenteredAtPosition ::
    Position lane ->
    [link] ->
    metricCodomain ->
    [LaneSegment lane]
  ballCenteredAtPosition position links radius =
    fuse . concatMap (neighborhoodOfPosition position links radius) $
      enumFrom minBound

  {-# MINIMAL neighborhoodOfPosition #-}

-- | This data type allows one to express the longitudinal connectivity between
-- two lanes, linking the first lane's exit with the entrance of the second
-- one, with both the exit and the entrance being such as to respect their
-- corresponding lanes' driving directions.
data LinkageTransition lane where
  LinkageT :: (Lane lane) => lane -> lane -> LinkageTransition lane

deriving instance (Eq lane) => Eq (LinkageTransition lane)

deriving instance (Ord lane) => Ord (LinkageTransition lane)

-- | 'linkageTReport' function generates a textual representation of a
-- 'LinkageTransition' value.
linkageTReport ::
  (FromBuilder b, Lane lane) =>
  (lane -> Builder) ->
  LinkageTransition lane ->
  b
linkageTReport laneBuilder (LinkageT l1 l2) =
  let indentation = replicate 32 ' '
   in fmt $
        laneBuilder l1
          |+ "\n"
          +| mconcat (replicate 3 (indentation |+ "\xFF5C\n"))
          +| indentation
          |+ "\\/\n"
          +| laneBuilder l2

-- | This instance of 'ProvidesNeighborhoodOfPosition' allows one to propagate a
-- given position longitudinally via instances of 'LinkageTransition', either
-- forward or backward, specified by a value of 'LaneDirection', and respecting
-- the driving direction of the lanes.
instance (Eq lane, Lane lane) => ProvidesNeighborhoodOfPosition (LinkageTransition lane) lane Double LaneDirection where
  neighborhoodOfPosition ::
    (Eq lane, Lane lane) =>
    Position lane ->
    [LinkageTransition lane] ->
    Double ->
    LaneDirection ->
    [LaneSegment lane]
  neighborhoodOfPosition (Pos lane sCoord) links dist dir
    | dist <= 0 = [LnSgmnt lane sCoord sCoord]
    | otherwise =
        let (distUntilExit, exit, shiftByDist) =
              if direction lane == dir
                then
                  ( longitudinalEnd lane - sCoord,
                    longitudinalEnd lane,
                    sCoord + dist
                  )
                else
                  ( sCoord - longitudinalStart lane,
                    longitudinalStart lane,
                    sCoord - dist
                  )

            nextPos =
              [ Pos nextLane nextSCoord
              | LinkageT fromLane toLane <- links,
                let (currentLane, nextLane) =
                      if dir == PositiveDirection
                        then (fromLane, toLane)
                        else (toLane, fromLane),
                let nextSCoord =
                      if dir == direction nextLane
                        then longitudinalStart nextLane
                        else longitudinalEnd nextLane,
                currentLane == lane
              ]

            nextLaneSegments =
              mconcat $
                map
                  ( \pos ->
                      neighborhoodOfPosition
                        pos
                        links
                        (dist - distUntilExit)
                        dir
                  )
                  nextPos
         in if distUntilExit > dist
              then return . fromJust $ mkLnSgmnt lane sCoord shiftByDist
              else
                fuse $
                  fromJust (mkLnSgmnt lane sCoord exit)
                    : nextLaneSegments

-- | The DefaultBehavior class enables associating two types by requiring the
-- second one to define a state space for the first one, thus making the latter
-- stateful.
class DefaultBehavior objectType stateSpace where
  act :: objectType -> stateSpace -> Double -> stateSpace
  -- ^ Given an object, its current state, and a Double value, the function
  -- shall deterministically determine the object's next state upon the time
  -- specified by the Double value elapses.

-- | The Action class allows considering values of a given type as actions by
-- specifying whether the resulting actions are optional or enforced.
class Action actionType where
  isForced :: actionType -> Bool
  -- ^ Given an action, the function shall indicate whether one shall take the
  -- action whenever feasible.

  description :: (FromBuilder b) => actionType -> b

-- | The Dynamics class specifies how applying an action affects some default
-- behavior.
class
  (Action actionType, DefaultBehavior objectType stateSpace) =>
  Dynamics actionType objectType stateSpace
  where
  applyAction :: objectType -> stateSpace -> actionType -> stateSpace
  -- ^ Given an action, an object, and its current state, the function shall
  -- indicate the object's next state upon executing the action.

  nextExecutionTime :: objectType -> stateSpace -> actionType -> Double
  -- ^ Given an action, an object, and its current state, the function shall
  -- indicate the soonest the object may perform the given action.

  {-# MINIMAL applyAction, nextExecutionTime #-}

-- | Given a stateful object and a corresponding state space, the Applicable
-- data type (parameterized by the object and state space) encapsulates
-- precisely those actions for which there is an instance of the Dynamics type
-- class with the specified object and state space.
data Applicable objectType stateSpace where
  Applicable ::
    forall actionType objectType stateSpace.
    ( Dynamics actionType objectType stateSpace,
      Typeable actionType,
      Eq actionType,
      Ord actionType
    ) =>
    actionType ->
    Applicable objectType stateSpace

decomposeApplicable ::
  (Typeable a) => Applicable objectType stateSpace -> Maybe a
decomposeApplicable (Applicable action) = cast action

instance Eq (Applicable objectType stateSpace) where
  (Applicable action1) == (Applicable action2) =
    Just action2 == cast action1

instance Ord (Applicable objectType stateSpace) where
  compare (Applicable action1) (Applicable action2) =
    maybe EQ (compare action1) (cast action2)

-- | Given a stateful object and its state space, the ActionApplication data
-- type provides a format for keeping a log of the applicable actions the given
-- object has taken, its outcome within the scope of the given state space, and
-- the relative time of the action's execution.
data ActionApplication objectType stateSpace
  = Applied (Applicable objectType stateSpace) stateSpace Double

deriving instance
  (Eq stateSpace) =>
  Eq (ActionApplication objectType stateSpace)

deriving instance
  (Ord stateSpace) =>
  Ord (ActionApplication objectType stateSpace)

applicationReport ::
  (FromBuilder b, DefaultBehavior objectType stateSpace) =>
  ActionApplication objectType stateSpace ->
  b
applicationReport (Applied (Applicable action) state time) =
  fmt $ "In " +| fixedF 2 time |+ " seconds, " +| description action +| ""

-- | Given a stateful object, a state from its state space, and a list of
-- actions applicable to the given object and state space, the getTrajectories
-- function simulates the object's behavior, returning a forest-like structure
-- of action applications.
getTrajectories ::
  (DefaultBehavior objectType stateSpace) =>
  objectType ->
  stateSpace ->
  [Applicable objectType stateSpace] ->
  Forest (ActionApplication objectType stateSpace)
getTrajectories _ _ [] = []
getTrajectories object state actions =
  let timeUntilForced =
        minimum
          [ nextExecutionTime object state action
          | Applicable action <- actions,
            isForced action
          ]
      stateWhenForced = act object state timeUntilForced
   in [ Node
          { rootLabel = Applied (Applicable action) newState time,
            subForest =
              getTrajectories
                object
                newState
                actions
          }
      | Applicable action <- actions,
        let forced = isForced action,
        let time = nextExecutionTime object state action,
        (forced && time == timeUntilForced)
          || (not forced && time < timeUntilForced),
        let intermediateState =
              if forced
                then stateWhenForced
                else act object state time,
        let newState = applyAction object intermediateState action
      ]

getStatesInTime ::
  (DefaultBehavior objectType stateSpace) =>
  objectType ->
  stateSpace ->
  Forest (ActionApplication objectType stateSpace) ->
  Double ->
  [stateSpace]
getStatesInTime object state [] time = [act object state time]
getStatesInTime object state actions time =
  let maxTime =
        maximum
          [ t
          | action <- actions,
            let Applied _ _ t = rootLabel action
          ]
      futureStates =
        if time <= maxTime
          then
            act object state time
              : mconcat
                [ getStatesInTime object state' actions' (time - t)
                | Node (Applied _ state' t) actions' <- actions,
                  t <= time
                ]
          else
            mconcat
              [ getStatesInTime object state' actions' (time - t)
              | Node (Applied _ state' t) actions' <- actions
              ]
   in futureStates
