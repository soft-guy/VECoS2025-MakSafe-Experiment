{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE ScopedTypeVariables #-}

module Main where

import Abstract
import Concrete
import Data.Map qualified as Map
import Data.Maybe (fromJust)
import System.Console.Terminal.Size (Window (..), size)

-- define the lanes
firstLane :: LaneV1 -- first lane
firstLane =
  LaneV1
    { rId1 = 1, -- road id
      longS1 = 0, -- start coordinate
      longE1 = 80, -- end coordinate
      lat1 = 1, -- lateral offset
      direction1 = PositiveDirection -- direction of the lane
    }

leftLane :: LaneV1 -- left lane of the second road
leftLane =
  LaneV1
    { rId1 = 2,
      longS1 = 0,
      longE1 = 200,
      lat1 = 1,
      direction1 = PositiveDirection
    }

rightLane :: LaneV1 -- right lane of the second road
rightLane =
  LaneV1
    { rId1 = 2,
      longS1 = 0,
      longE1 = 200,
      lat1 = -1,
      direction1 = PositiveDirection
    }

-- define linkage transitions
linkageFirstLeft :: LinkageTransition LaneV1
linkageFirstLeft = LinkageT firstLane leftLane

linkageFirstRight :: LinkageTransition LaneV1
linkageFirstRight = LinkageT firstLane rightLane

links :: [LinkageTransition LaneV1]
links = [linkageFirstLeft, linkageFirstRight]

-- define positions
position1_ego :: Position LaneV1 -- ego vehicle's position
position1_ego = fromJust $ mkPosition firstLane 78

position2_second :: Position LaneV1 -- second vehicle's position
position2_second = fromJust $ mkPosition leftLane 2

position3_third :: Position LaneV1 -- third vehicle's position
position3_third = fromJust $ mkPosition rightLane 1

-- define vehicles
vehicle1Ego :: Vehicle -- ego vehicle
vehicle1Ego =
  Vehicle
    { vId = 1, -- vehicle id
      vName = "Ego", -- vehicle name
      longDim = 2, -- length of the vehicle
      spMax = 5
    }

vehicle2_second :: Vehicle -- second vehicle
vehicle2_second = Vehicle {vId = 2, vName = "left", longDim = 2, spMax = 5}

vehicle3_third :: Vehicle -- third vehicle
vehicle3_third = Vehicle {vId = 3, vName = "right", longDim = 2, spMax = 5}

-- define vehicle states
vehicleState1_ego :: VehicleState LaneV1 -- ego vehicle's initial state
vehicleState1_ego =
  VehicleState
    position1_ego
    3 -- speed of the ego vehicle
    3 -- acceleration of the ego vehicle
    PositiveDirection
    []
    []

vehicleState2_second :: VehicleState LaneV1 -- second vehicle's initial state
vehicleState2_second =
  VehicleState
    position2_second
    2 -- speed of the second vehicle
    0 -- acceleration of the second vehicle
    PositiveDirection
    []
    []

vehicleState3_third :: VehicleState LaneV1
vehicleState3_third =
  VehicleState
    position3_third
    5 -- speed of the third vehicle
    1 -- acceleration of the third vehicle
    PositiveDirection
    []
    []

-- initial configuration (scene)
config :: VehicleConfiguration LaneV1
config =
  VehicleConfiguration
    vehicle1Ego
    vehicleState1_ego
    ( Map.fromList
        [ (vehicle2_second, vehicleState2_second),
          (vehicle3_third, vehicleState3_third)
        ]
    )

-- find the front vehicles over the threshold of 2 seconds for the ego vehicle
frontVs :: [((Vehicle, VehicleTrajectory LaneV1), Double)]
frontVs = frontVehicle config vehicle1Ego links 2

closeVehicles = distanceFrontGEqS config vehicle1Ego 2 links

main :: IO ()
main = do
  putStrLn "Front Vehicles for Ego Vehicle over 2 seconds threshold:"
  frontVehiclesReport frontVehicleReport frontVs
  mWindow <- size
  let termWidth = maybe 80 width mWindow -- fallback to width 80
  putStrLn (replicate termWidth '-')
  putStrLn "Front Vehicles closer than 2 seconds for Ego Vehicle:"
  distanceEgoFrontGEqSReportMany
    vehicleReport
    applicationReport
    closeVehicles
