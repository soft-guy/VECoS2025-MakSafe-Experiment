{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE ScopedTypeVariables #-}

module Concrete where

import Abstract
import Control.Monad (join)
import Data.Function (on)
import Data.List
import Data.Map qualified as Map
import Data.Maybe
import Data.Ord
import Data.Text (Text)
import Data.Text qualified as T
import Data.Text.IO qualified as TIO
import Data.Text.Lazy.Builder (fromText)
import Data.Tree (Forest, Tree (Node, rootLabel))
import Data.Typeable (Typeable, cast)
import Fmt
import Fmt.Internal.Core (FromBuilder)

round2dp :: Double -> Double
round2dp x = fromIntegral (round $ x * 1e2) / 1e2

data LaneV1 = LaneV1
  { rId1 :: Int,
    longS1 :: Double,
    longE1 :: Double,
    lat1 :: Int,
    direction1 :: LaneDirection
  }
  deriving (Eq, Ord, Read)

instance Lane LaneV1 where
  roadId = rId1
  longitudinalStart = longS1
  lateralOffset = lat1
  direction = direction1
  longitudinalEnd = longE1

data Vehicle = Vehicle
  { vId :: Int,
    vName :: Text,
    longDim :: Double,
    spMax :: Double
  }
  deriving (Ord)

instance Eq Vehicle where
  (==) :: Vehicle -> Vehicle -> Bool
  (Vehicle id1 _ _ _) == (Vehicle id2 _ _ _) = id1 == id2

vehicleReport :: (FromBuilder b) => Vehicle -> b
vehicleReport v =
  fmt $ "Vehicle \"" +| vName v |+ "\""

data VehicleState lane where
  VehicleState ::
    (Lane lane) =>
    -- | reference point's position
    Position lane ->
    -- | longitudinal speed
    Double ->
    -- | longitudinal acceleration (deceleration)
    Double ->
    -- | longitudinal movement direction wrt the lane
    LaneDirection ->
    -- | assuming lanes
    [lane] ->
    -- | leaving lanes
    [lane] ->
    VehicleState lane

getDirection :: VehicleState lane -> LaneDirection
getDirection (VehicleState _ _ _ dir _ _) = dir

deriving instance Eq (VehicleState lane)

deriving instance Ord (VehicleState lane)

timeAccMax :: Vehicle -> VehicleState lane -> Double
timeAccMax v (VehicleState _ sp acc _ _ _)
  | acc == 0 = read "Infinity"
  | acc < 0 = abs $ sp / acc
  | acc > 0 = (spMax v - sp) / acc

distWhileAcc :: VehicleState lane -> Double -> Double
distWhileAcc (VehicleState _ sp acc _ _ _) t = sp * t + 0.5 * acc * t ^ 2

distAfterAcc :: Vehicle -> VehicleState lane -> Double -> Double
distAfterAcc v (VehicleState _ _ acc _ _ _) t =
  if acc < 0
    then 0
    else spMax v * t

distanceTime ::
  (Lane lane) =>
  Vehicle ->
  VehicleState lane ->
  Double ->
  Double
distanceTime v vs@(VehicleState _ sp acc _ _ _) t =
  let timeAccMax' = timeAccMax v vs
   in if t > timeAccMax'
        then distWhileAcc vs timeAccMax' + distAfterAcc v vs (t - timeAccMax')
        else distWhileAcc vs t

timeDistance ::
  (Lane lane) =>
  Vehicle ->
  VehicleState lane ->
  Double ->
  Double
timeDistance v vs@(VehicleState _ sp 0 _ _ _) d = d / sp
timeDistance v vs@(VehicleState _ sp acc _ _ _) d =
  let timeAccMax' = timeAccMax v vs
      distWhileAcc' = distWhileAcc vs timeAccMax'
   in if d > distWhileAcc'
        then
          let t_after_acc = (d - distWhileAcc') / spMax v
           in if acc < 0
                then read "Infinity"
                else timeAccMax' + t_after_acc
        else
          let a = 0.5 * acc
              b = sp
              c = -d
              discriminant = b ^ 2 - 4 * a * c
           in (-b + sqrt discriminant) / (2 * a)

vehicleStateReport ::
  (FromBuilder b) => (Position lane -> Builder) -> VehicleState lane -> b
vehicleStateReport
  posBuilder
  (VehicleState pos speed acc dir _ _) =
    fmt $
      "located at "
        +| posBuilder pos
        |+ "; traveling at "
        +| speed
        |+ " m/s"
        +| ( if acc < 0
               then
                 " and braking at " +| abs acc |+ " m/s^2"
               else
                 if acc > 0
                   then " and accelerating at " +| acc |+ " m/s^2"
                   else
                     ""
           )
        +| if dir == PositiveDirection
          then ""
          else
            "\nAttention! The vehicle is driving against the flow of"
              +| " traffic!"

instance DefaultBehavior Vehicle (VehicleState lane) where
  act :: Vehicle -> VehicleState lane -> Double -> VehicleState lane
  act v s@(VehicleState (Pos lane c) sp acc dir a l) t
    | t <= 0 = s
    | otherwise =
        let newSp =
              if acc < 0
                then max 0 (sp + acc * t)
                else min (spMax v) (sp + acc * t)

            endPoint =
              if dir == direction lane
                then longitudinalEnd lane
                else longitudinalStart lane

            t' = timeAccMax v s
            newAcc = if t >= t' then 0 else acc

            distanceToEnd = abs (endPoint - c)

            d = distanceTime v s t
            d_signed = if dir == direction lane then d else -d
         in if d <= distanceToEnd
              then VehicleState (Pos lane (c + d_signed)) newSp newAcc dir a l
              else VehicleState (Pos lane endPoint) 0 0 dir a l

--  | Returns the zone comprising the lanes the vehicle is currently present in.
vehicleLanesZone ::
  (Eq lane) =>
  VehicleState lane ->
  Zone lane
vehicleLanesZone (VehicleState (Pos lane _) _ _ _ a l) =
  let zones = map laneZone (lane : a ++ l)
   in foldl' zoneUnion [] zones

collisionZone ::
  (Eq lane) =>
  Vehicle ->
  VehicleState lane ->
  [LinkageTransition lane] ->
  Zone lane
collisionZone
  (Vehicle _ _ longDim _)
  state@(VehicleState pos _ _ _ _ _)
  links =
    let zone1 = ballCenteredAtPosition pos links (longDim / 2)
        zone2 = vehicleLanesZone state
     in zoneIntersection zone1 zone2

getAnteriorLane :: (Lane lane) => VehicleState lane -> lane
getAnteriorLane (VehicleState (Pos lane _) _ _ _ assuming _) =
  last $ lane : assuming

getRearmostLane :: (Lane lane) => VehicleState lane -> lane
getRearmostLane (VehicleState (Pos lane _) _ _ _ _ leaving) =
  case leaving of
    [] -> lane
    l : _ -> l

reachabilityZone ::
  (Eq lane) =>
  Vehicle ->
  VehicleState lane ->
  [LinkageTransition lane] ->
  Double ->
  Zone lane
reachabilityZone
  vehicle
  state@(VehicleState _ speed acc dir _ _)
  links
  t
    | t < 0 = []
    | otherwise =
        let anteriorLane = getAnteriorLane state

            anteriorLaneZ = laneZone anteriorLane

            collisionZ = collisionZone vehicle state links

            anteriorLaneSegmentZ =
              zoneIntersection
                collisionZ
                anteriorLaneZ

            timeAccMax
              | acc == 0 = read "Infinity"
              | acc < 0 = abs $ speed / acc
              | acc > 0 = (spMax vehicle - speed) / acc

            distWhileAcc t' = speed * t' + 0.5 * acc * t' ^ 2
            distAfterAcc t' = if acc < 0 then 0 else spMax vehicle * t'
            d =
              if t > timeAccMax
                then distWhileAcc timeAccMax + distAfterAcc (t - timeAccMax)
                else distWhileAcc t

            (LnSgmnt _ s e) =
              case listToMaybe anteriorLaneSegmentZ of
                Just segment -> segment
                Nothing -> error "No intersection found"
            coord' = if direction anteriorLane == dir then e else s
            reachableZ =
              neighborhoodOfPosition
                (Pos anteriorLane coord')
                links
                d
                dir
         in zoneUnion collisionZ reachableZ

data VehicleConfiguration lane where
  VehicleConfiguration ::
    -- | ego vehicle
    Vehicle ->
    -- | ego vehicle's state
    VehicleState lane ->
    -- | other vehicles and their states
    Map.Map Vehicle (VehicleState lane) ->
    VehicleConfiguration lane

vehicleConfigurationReport ::
  (FromBuilder b) =>
  (Vehicle -> Builder) ->
  (VehicleState lane -> Builder) ->
  (Position lane -> Builder) ->
  VehicleConfiguration lane ->
  b
vehicleConfigurationReport
  vBuilder
  vSBuilder
  pBuilder
  (VehicleConfiguration ego egoState others) =
    fmt
      $ blockListF'
        "-"
        ( \(v, s) ->
            nameF
              (vBuilder v +| (if v == ego then " (ego)" else ""))
              (vSBuilder s)
        )
      $ (ego, egoState) : Map.toList others

data AdoptLane lane where
  AdoptLinkageT ::
    (Lane lane, Typeable lane) =>
    LinkageTransition lane ->
    AdoptLane lane

deriving instance Eq (AdoptLane lane)

deriving instance Ord (AdoptLane lane)

adoptLaneReport :: (FromBuilder p) => (lane -> Builder) -> AdoptLane lane -> p
adoptLaneReport builder (AdoptLinkageT lt) =
  fmt $ nameF "adopt linkage transition" $ linkageTReport builder lt

instance (Lane lane) => Action (AdoptLane lane) where
  isForced (AdoptLinkageT _) = True
  description = adoptLaneReport laneReport'

instance
  (Lane lane) =>
  Dynamics (AdoptLane lane) Vehicle (VehicleState lane)
  where
  nextExecutionTime
    v@(Vehicle _ _ longDim _)
    state@(VehicleState (Pos lane c) _ _ dir a _)
    (AdoptLinkageT (LinkageT l1 l2)) =
      let anteriorLane = getAnteriorLane state
          dist1 =
            if direction lane == dir
              then longitudinalEnd lane - c
              else c - longitudinalStart lane
          dist2 = sum $ map longitudinalLength a
          t = timeDistance v state (dist1 + dist2 - longDim / 2)
       in if (dir == PositiveDirection && l1 == anteriorLane)
            || (dir == NegativeDirection && l2 == anteriorLane)
            then t
            else read "Infinity"
  applyAction
    _
    (VehicleState pos sp acc dir a l)
    (AdoptLinkageT (LinkageT l1 l2)) =
      let newLane = if dir == PositiveDirection then l2 else l1
       in VehicleState pos sp acc dir (a ++ [newLane]) l

getAdoptLaneActions ::
  forall a b lane.
  (Lane lane, Typeable lane, Dynamics (AdoptLane lane) a b) =>
  [LinkageTransition lane] ->
  [Applicable a b]
getAdoptLaneActions = map (Applicable . AdoptLinkageT)

data LeaveLane lane where
  LeaveLong :: (Lane lane) => lane -> LeaveLane lane

deriving instance Eq (LeaveLane lane)

deriving instance Ord (LeaveLane lane)

leaveLaneReport :: (FromBuilder p) => (lane -> Builder) -> LeaveLane lane -> p
leaveLaneReport builder (LeaveLong lane) = fmt $ "Leave " +| builder lane

instance (Lane lane) => Action (LeaveLane lane) where
  isForced (LeaveLong _) = True
  description = leaveLaneReport laneReport'

instance
  (Lane lane) =>
  Dynamics (LeaveLane lane) Vehicle (VehicleState lane)
  where
  nextExecutionTime
    v@(Vehicle _ _ longDim _)
    st@(VehicleState (Pos lane c) _ _ dir _ l)
    (LeaveLong laneToLeave) =
      case l of
        [] -> read "Infinity"
        (endingLane : _) ->
          let dist1 =
                if direction lane == dir
                  then c - longitudinalStart lane
                  else longitudinalEnd lane - c
              dist2 = sum $ map longitudinalLength l
              endingLaneL = longitudinalLength endingLane
              t =
                timeDistance
                  v
                  st
                  (endingLaneL - (dist1 + dist2 - longDim / 2))
           in if laneToLeave == endingLane
                then t
                else read "Infinity"
  applyAction
    _
    (VehicleState pos sp acc dir a l)
    _ = VehicleState pos sp acc dir a (drop 1 l)

getLeaveLaneActions ::
  (Lane lane, Typeable lane) =>
  [LinkageTransition lane] ->
  ( forall a b.
    (Dynamics (LeaveLane lane) a b) =>
    [Applicable a b]
  )
getLeaveLaneActions linkage =
  let uniqueLanes =
        nub $ concatMap (\(LinkageT l1 l2) -> [l1, l2]) linkage
   in map (Applicable . LeaveLong) uniqueLanes

data ExecuteLinkageTransition lane where
  ExecuteLinkageTransition ::
    (Lane lane) =>
    lane ->
    ExecuteLinkageTransition lane

deriving instance Eq (ExecuteLinkageTransition lane)

deriving instance Ord (ExecuteLinkageTransition lane)

executeLinkageTReport ::
  (FromBuilder p) =>
  (lane -> Builder) ->
  ExecuteLinkageTransition lane ->
  p
executeLinkageTReport builder (ExecuteLinkageTransition lane) =
  fmt $
    "The ego vehicle's reference point's new lane is " +| builder lane

instance (Lane lane) => Action (ExecuteLinkageTransition lane) where
  isForced (ExecuteLinkageTransition _) = True
  description = executeLinkageTReport laneReport'

instance
  (Lane lane) =>
  Dynamics (ExecuteLinkageTransition lane) Vehicle (VehicleState lane)
  where
  nextExecutionTime
    v
    state@(VehicleState (Pos lane c) _ _ dir a _)
    (ExecuteLinkageTransition nextL) =
      case a of
        [] -> read "Infinity"
        (nextLane : _) ->
          let dist =
                if direction lane == dir
                  then longitudinalEnd lane - c
                  else c - longitudinalStart lane
           in if nextLane == nextL
                then timeDistance v state dist
                else read "Infinity"
  applyAction
    _
    (VehicleState (Pos lane _) sp acc dir (newLane : rest) l)
    _ =
      let newC =
            if direction newLane == dir
              then longitudinalStart newLane
              else longitudinalEnd newLane
       in VehicleState (Pos newLane newC) sp acc dir rest (l ++ [lane])
  applyAction _ state _ = state

getExecuteLTsActions ::
  (Lane lane, Typeable lane) =>
  [LinkageTransition lane] ->
  ( forall a b.
    (Dynamics (ExecuteLinkageTransition lane) a b) =>
    [Applicable a b]
  )
getExecuteLTsActions links =
  let uniqueLanes =
        nub $ concatMap (\(LinkageT l1 l2) -> [l1, l2]) links
   in map (Applicable . ExecuteLinkageTransition) uniqueLanes

getBasicLongitudinalActions ::
  (Lane lane, Typeable lane) =>
  [LinkageTransition lane] ->
  [Applicable Vehicle (VehicleState lane)]
getBasicLongitudinalActions links =
  let adoptLaneActions = getAdoptLaneActions links
      leaveLaneActions = getLeaveLaneActions links
      executeLTsActions = getExecuteLTsActions links
   in adoptLaneActions ++ leaveLaneActions ++ executeLTsActions

doesOccupyLaneAtTime ::
  (Eq lane) =>
  Vehicle ->
  VehicleState lane ->
  Forest (ActionApplication Vehicle (VehicleState lane)) ->
  Double ->
  lane ->
  [LinkageTransition lane] ->
  Maybe (LaneSegment lane)
doesOccupyLaneAtTime vehicle initState trajectories time lane links =
  find (\(LnSgmnt l _ _) -> l == lane) $
    fuse $
      concatMap
        (\state -> collisionZone vehicle state links)
        (getStatesInTime vehicle initState trajectories time)

anteriorLaneTree ::
  (Lane lane) =>
  Forest (ActionApplication Vehicle (VehicleState lane)) ->
  Forest (lane, Double)
anteriorLaneTree = fmap (fmap extractAnteriorLane)
  where
    extractAnteriorLane (Applied _ state time) =
      (getAnteriorLane state, time)

isAnteriorLaneBetween ::
  (Lane lane) =>
  Forest (ActionApplication Vehicle (VehicleState lane)) ->
  Double ->
  Double ->
  lane ->
  Maybe Double
isAnteriorLaneBetween actions time1 time2 lane =
  let go acc t1 t2 (Node (l, t) children)
        | t > t2 = []
        | t < t1 = concatMap (go (acc + t) (t1 - t) (t2 - t)) children
        | l == lane = [acc + t]
        | otherwise = concatMap (go (acc + t) 0 (t2 - t)) children
      times = concatMap (go 0 time1 time2) (anteriorLaneTree actions)
   in listToMaybe $ sort times

type VehicleTrajectory lane = [ActionApplication Vehicle (VehicleState lane)]

frontVehicleCase1 ::
  (Lane lane) =>
  -- | Vehicle configuration
  Map.Map Vehicle (VehicleState lane) ->
  -- | Vehicle relative to which we are looking for the front vehicle
  Vehicle ->
  -- | Linkage transitions
  [LinkageTransition lane] ->
  [Vehicle]
frontVehicleCase1 allVehiclesStates vehicle linkageTs =
  let vehicleState = Map.lookup vehicle allVehiclesStates

      vehicleDirection =
        (\(VehicleState _ _ _ dir _ _) -> dir) <$> vehicleState

      vehicleALane = getAnteriorLane <$> vehicleState

      vehicleALnSgmnt =
        ( listToMaybe
            =<< ( zoneIntersection
                    <$> ( collisionZone
                            vehicle
                            <$> vehicleState
                            <*> pure linkageTs
                        )
                    <*> (laneZone <$> vehicleALane)
                )
        )

      vehiclesStates = Map.delete vehicle allVehiclesStates

      maybeFrontVehicles =
        Map.mapMaybeWithKey
          ( \v s ->
              let sameALane = Just (getAnteriorLane s) == vehicleALane
                  intersection =
                    ( ( listToMaybe
                          <$> zoneIntersection
                            (collisionZone v s linkageTs)
                      )
                        . laneZone
                        <$> vehicleALane
                    )
               in if sameALane then intersection else Nothing
          )
          vehiclesStates

      closestPositions =
        let (cmp, endPoint1, endPoint2, sortFn) =
              case vehicleDirection == (direction <$> vehicleALane) of
                True -> ((<=), getEndLnSgmnt, getStartLnSgmnt, sortOn snd)
                False ->
                  ((>=), getStartLnSgmnt, getEndLnSgmnt, sortOn (Down . snd))

            vehicleEndPos = endPoint1 <$> vehicleALnSgmnt

            filtered =
              Map.filter
                (\c -> maybe False (`cmp` c) vehicleEndPos)
                $ Map.mapMaybe (fmap endPoint2) maybeFrontVehicles
         in sortFn (Map.toList filtered)

      closestVehicles =
        fmap (map fst) $
          listToMaybe $
            groupBy (\a b -> snd a == snd b) closestPositions
   in fromMaybe [] closestVehicles

frontVehicle ::
  forall lane.
  (Lane lane, Typeable lane) =>
  -- | Vehicle configuration
  VehicleConfiguration lane ->
  -- | Vehicle relative to which we are looking for the front vehicle
  Vehicle ->
  -- | Linkage transitions
  [LinkageTransition lane] ->
  Double ->
  [((Vehicle, VehicleTrajectory lane), Double)]
frontVehicle
  (VehicleConfiguration ego egoState others)
  vehicle
  links
  timeHorizon =
    let allVehiclesStates = Map.insert ego egoState others
        vehicleState = Map.lookup vehicle allVehiclesStates
        vehiclesStates = Map.delete vehicle allVehiclesStates
        counterDirectedVehicles =
          Map.filter
            ( \s ->
                maybe False ((/=) (getDirection s) . getDirection) vehicleState
            )
            vehiclesStates

        actions = getBasicLongitudinalActions links

        vehicleTrajectories =
          getTrajectories
            vehicle
            <$> vehicleState
            <*> pure actions

        vehiclesTrajectories =
          Map.mapWithKey
            (\v s -> getTrajectories v s actions)
            vehiclesStates

        cDVehiclesTrajectories =
          Map.mapWithKey
            (\v s -> getTrajectories v s actions)
            counterDirectedVehicles

        iterationOne prefix vehicleConfiguration time
          | time > timeHorizon = []
          | otherwise =
              let case1 =
                    map ((,time) . (,prefix)) $
                      frontVehicleCase1
                        vehicleConfiguration
                        vehicle
                        links

                  nextAnteriorLanes =
                    let processOneTree
                          ( Node
                              application@(Applied action state t)
                              children
                            )
                            | t < time = concatMap processOneTree children
                            | t > timeHorizon = []
                            | time <= t && t <= timeHorizon =
                                if isJust
                                  ( decomposeApplicable action ::
                                      Maybe (AdoptLane lane)
                                  )
                                  then return ((state, t), application)
                                  else concatMap processOneTree children
                            | otherwise = []
                     in concatMap processOneTree <$> vehicleTrajectories

                  laneChangeTimes =
                    map listToMaybe . group . sort . map (snd . fst)
                      <$> nextAnteriorLanes

                  case2 =
                    concatMap
                      ( \t' ->
                          Map.toList $
                            Map.mapMaybe
                              ( \trajectory ->
                                  join $
                                    isAnteriorLaneBetween
                                      trajectory
                                      time
                                      <$> t'
                                      <*> ( getAnteriorLane
                                              <$> Map.lookup
                                                vehicle
                                                vehicleConfiguration
                                          )
                              )
                              cDVehiclesTrajectories
                      )
                      <$> laneChangeTimes

                  case2' =
                    map
                      (\(l, t) -> ((vehicle, prefix), t))
                      <$> case2

                  nextConfigs =
                    concatMap
                      ( \((state, time), action) ->
                          map
                            ((,time) . (,action))
                            ( sequence
                                ( Map.mapWithKey
                                    ( \v s ->
                                        if v == vehicle
                                          then
                                            [state]
                                          else
                                            getStatesInTime v s (getTrajectories v s actions) time
                                    )
                                    vehicleConfiguration
                                )
                            )
                      )
                      <$> nextAnteriorLanes
                  case3 =
                    concatMap
                      ( \((newConfig, action), t) ->
                          iterationOne
                            (prefix ++ [action])
                            newConfig
                            t
                      )
                      <$> nextConfigs
               in case () of
                    _
                      | not (null case1) -> case1
                      | Just c2 <- case2', not (null c2) -> c2
                      | Just c3 <- case3 -> c3
                      | otherwise -> []
     in iterationOne [] allVehiclesStates 0

frontVehicleReport ::
  (FromBuilder b) => ((Vehicle, VehicleTrajectory lane), Double) -> b
frontVehicleReport ((v, tr), t) =
  fmt $
    vehicleReport v
      +| " is the front vehicle when "
      +| fixedF 2 t
      |+ " seconds elapse. "
      +| if not $ null tr
        then
          nameF
            "The vehicle of interest trajectory"
            (blockListF' "$" applicationReport tr)
        else ""

frontVehiclesReport ::
  (FromBuilder b) =>
  (((Vehicle, VehicleTrajectory lane), Double) -> Builder) ->
  [((Vehicle, VehicleTrajectory lane), Double)] ->
  b
frontVehiclesReport builder = fmt . blockListF' "*" builder

distanceFrontGEqS ::
  (Lane lane, Typeable lane) =>
  VehicleConfiguration lane ->
  Vehicle ->
  Double ->
  [LinkageTransition lane] ->
  ([(Vehicle, VehicleTrajectory lane)], Double)
distanceFrontGEqS
  config@(VehicleConfiguration ego egoState others)
  vehicle@(Vehicle _ _ _ spMax)
  t
  links =
    let allVehiclesStates = Map.insert ego egoState others
        frVs = frontVehicle config vehicle links t

        checkCollisionWithFront
          vSt@(VehicleState _ sp acc dir _ _)
          front@(Vehicle _ _ _ frSpMax)
          frSt@(VehicleState _ frSp frAcc frDir _ _)
          residualTime =
            let timeWhileAccV = timeAccMax vehicle vSt
                timeWhileAccFr = timeAccMax front frSt

                frontTrajectories =
                  getTrajectories front frSt (getBasicLongitudinalActions links)
                reachabilityZoneFr =
                  reachabilityZone front frSt links residualTime
                reachabilityZoneV = reachabilityZone vehicle vSt links residualTime

                constV = if acc > 0 then spMax else 0
                constFr = if frAcc > 0 then frSpMax else 0

                t1 =
                  let t1' = (sp - frSp) / (frAcc - acc)
                   in if (frAcc - acc > 0) && t1' > 0
                        then
                          Just $
                            minimum
                              [ t1',
                                timeWhileAccV,
                                timeWhileAccFr,
                                residualTime
                              ]
                        else
                          ( if ( (frAcc - acc < 0)
                                   && t1' < min timeWhileAccV timeWhileAccFr
                               )
                              || (frAcc == acc && sp > frSp)
                              then Just $ min timeWhileAccV timeWhileAccFr
                              else Nothing
                          )

                t21 =
                  let t21' = (constV - frSp) / frAcc
                   in if frAcc > 0 && t21' > timeWhileAccV
                        then Just $ min timeWhileAccFr t21'
                        else
                          ( if (frAcc < 0 && t21' < timeWhileAccFr)
                              || (frAcc == 0 && constV > frSp)
                              then Just timeWhileAccFr
                              else Nothing
                          )

                t22 =
                  let t22' = (constFr - sp) / acc
                   in (if (acc > 0 && t22' < timeWhileAccV) || (acc == 0 && sp > constFr) then Just timeWhileAccV else (if acc < 0 && t22' > timeWhileAccFr then Just $ min timeWhileAccV t22' else Nothing))

                t2 = if timeWhileAccV <= timeWhileAccFr then t21 else t22

                t3 =
                  if constV > constFr
                    && max timeWhileAccV timeWhileAccFr < read "Infinity"
                    then Just $ read "Infinity"
                    else Nothing

                times =
                  nub $
                    map (`min` residualTime) $
                      nub $
                        catMaybes [t1, t2, t3]
                checkTime t =
                  let statesFr =
                        getStatesInTime
                          front
                          frSt
                          frontTrajectories
                          t

                      reachabilityZoneV = reachabilityZone vehicle vSt links t

                      collisionZonesFr =
                        map (\st -> collisionZone front st links) statesFr
                   in any
                        ( \z ->
                            not $
                              null $
                                zoneIntersection z reachabilityZoneV
                        )
                        collisionZonesFr
                checkTimes = any checkTime times
             in if dir /= frDir then not $ null $ zoneIntersection reachabilityZoneV reachabilityZoneFr else checkTimes
        checkVehicle ((frV, tr), t') =
          let actions = getBasicLongitudinalActions links

              vSt = Map.lookup vehicle allVehiclesStates

              frVSt = Map.lookup frV allVehiclesStates

              vStatesAtT' = do
                vSt' <- vSt
                return $
                  if null tr
                    then
                      getStatesInTime
                        vehicle
                        vSt'
                        (getTrajectories vehicle vSt' actions)
                        t'
                    else
                      let Applied _ intermediateState t'' = last tr
                       in getStatesInTime
                            vehicle
                            intermediateState
                            (getTrajectories vehicle intermediateState actions)
                            (t' - t'')

              frVTrajectories =
                do
                  frVSt' <- frVSt
                  return $ getTrajectories frV frVSt' actions

              frVStatesAtT' =
                do
                  frVSt' <- frVSt
                  frVTrajectories' <- frVTrajectories
                  return $ getStatesInTime frV frVSt' frVTrajectories' t'
           in case liftA2 (,) <$> vStatesAtT' <*> frVStatesAtT' of
                Just pairs ->
                  if any
                    ( \(vState, frVState) ->
                        checkCollisionWithFront vState frV frVState (t - t')
                    )
                    pairs
                    then return (frV, tr)
                    else []
                Nothing -> []
     in (concatMap checkVehicle frVs, t)

distanceEgoFrontGEqSReport ::
  (FromBuilder b) =>
  (Vehicle -> Builder) ->
  (ActionApplication Vehicle (VehicleState lane) -> Builder) ->
  Double ->
  Vehicle ->
  VehicleTrajectory lane ->
  b
distanceEgoFrontGEqSReport vB applicationB t v tr =
  fmt $
    "The distance from the front vehicle ["
      +| vB v
      +| "] is less than "
      +| t
      |+ " seconds."
      +| if not $ null tr
        then
          nameF
            "\nThe vehicle of interest trajectory"
            (blockListF' "$" applicationB tr)
        else ""

distanceEgoFrontGEqSReportMany ::
  (FromBuilder b) =>
  (Vehicle -> Builder) ->
  (ActionApplication Vehicle (VehicleState lane) -> Builder) ->
  ([(Vehicle, VehicleTrajectory lane)], Double) ->
  b
distanceEgoFrontGEqSReportMany _ _ ([], t) =
  fmt $ "No front vehicles closer than " +| t |+ " seconds."
distanceEgoFrontGEqSReportMany vB applicationB (vs, t) =
  fmt $
    blockListF' "@" (uncurry $ distanceEgoFrontGEqSReport vB applicationB t) vs
