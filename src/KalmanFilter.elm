module KalmanFilter
    exposing
        ( Model
        , Params
        , State
        , defaultParams
        , init
        , fromMeasurement
        , filter
        , filterWithNoiseEstimates
        , filterMeasurement
        , predictNext
        )

{-| Simple 1D Kalman Filters in Elm.
Kalman filters are great for estimating true values of sequenced noisy data.
For example, it can be useful for calculating and displaying moving averages in
otherwise noisy data. Kalman filters are generalizable to work in N dimensions,
however this library is only equipped to deal with 1D data.

For more information on the theory of the Kalman Filter, check out [a great
intro paper here](http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

# Basic Usage

These are the simplest way to get started using Kalman filters. You just apply
the filter to the list and it works. Voila!~~

@docs filter, filterWithNoiseEstimates

# Advanced Usage

These functions should be used if you don't like the way `filter` does
stuff, or you need your implementation to be on-line (with rolling updates)

## Model
@docs Model, Params, State, defaultParams, init, fromMeasurement

## Functions
@docs filterMeasurement, predictNext
-}

-- Exposed Module Code


{-| Current state of the filter.
The `prediction` field holds what the filter thinks the last value learned from
'should' have been. The `covariance` field is for internal use.
-}
type alias State =
    { prediction : Float
    , covariance : Float
    }


{-| Parameters for the kalman filter. Use these to tune the filter for your
use case.
-}
type alias Params =
    { expectedNoisePower : Float
    , desiredNoisePower : Float
    , stateFactor : Float
    , controlFactor : Float
    , measurementFactor : Float
    }


{-| Decent default parameters for a kalman filter. It is highly recommended
that you fine-tune this for a better convergence / training time balance.
-}
defaultParams : Params
defaultParams =
    { expectedNoisePower = 1
    , desiredNoisePower = 1
    , stateFactor = 1
    , controlFactor = 0
    , measurementFactor = 1
    }


{-| Holds the kalman filter state and parameters.
-}
type alias Model =
    { state : State, params : Params }


{-| Initializes a kalman filter with an initial prediction of 0 and a
covariance of 1. If possible, it is recommended to use `fromMeasurement`
to instantiate a `KalmanFilter.Model` as it will converge much faster.
-}
init : Maybe Params -> Model
init optionalParams =
    let
        params =
            optionalParams |> Maybe.withDefault defaultParams

        state =
            { prediction = 0, covariance = 1 }
    in
        { state = state, params = params }


{-| Initialize a kalman filter from a measurement. This is the recommended way
to instantiate a `KalmanFilter.Model` as it converges much faster than when
starting at 0.
-}
fromMeasurement : Maybe Params -> Float -> Model
fromMeasurement optionalParams measurement =
    let
        model =
            init optionalParams

        inverseMeasurementFactor =
            1 / model.params.measurementFactor

        currState =
            model.state
    in
        { model
            | state =
                { currState
                    | prediction = inverseMeasurementFactor * measurement
                    , covariance =
                        ((inverseMeasurementFactor ^ 2)
                            * model.params.expectedNoisePower
                        )
                }
        }


{-| Filters a noisy signal, providing the predicted 'true' value of
the signal. Makes working with kalman filters very easy.
-}
filter : Maybe Params -> List Float -> List Float
filter optionalParams noisySignal =
    let
        allNothing =
            noisySignal |> List.map (always Nothing)
    in
        List.map2 (,) noisySignal allNothing
            |> filterMaybeNoiseEstimates optionalParams


{-| Like `filter` but including noise estimation as the second item in
the tuple of examples. If not appliccable for some signals, `0` can be supplied
as the noiseEstimation.
-}
filterWithNoiseEstimates : Maybe Params -> List ( Float, Float ) -> List Float
filterWithNoiseEstimates optionalParams noisySignalTuple =
    noisySignalTuple
        |> List.map (Tuple.mapSecond Just)
        |> filterMaybeNoiseEstimates optionalParams


{-| Run a measurement through a filter with an optional `controlFactor`,
returning the model with its parameters updated to reflect the measurement
recording.

What the filter thinks the maximum likelihood for the 'true' value of the last
measurement should be can be accessed in the resulting `model.state.prediction`

`maybeNoiseEstimation` may be provided to if an indicator of what the noise
for this measurement is able to be provided. For example, if using the most
likely value from a probability density histogram, the probability of the value
inversely compared to the probability of the rest may be a good indicator of
the amount of noise associated with a measurement.
-}
filterMeasurement : Model -> Maybe Float -> Float -> Model
filterMeasurement model maybeNoiseEstimation measurement =
    predictNext model maybeNoiseEstimation
        |> learn model measurement


{-| Predict the next value in the series.
This is a prediction about what it thinks the 'true' value of the next element
should be before seeing the next measurement.
-}
predictNext : Model -> Maybe Float -> Float
predictNext model maybeNoiseEstimation =
    let
        controlFactor =
            maybeNoiseEstimation |> Maybe.withDefault 0
    in
        (model.params.stateFactor * model.state.prediction)
            + (model.params.controlFactor * controlFactor)


learn : Model -> Float -> Float -> Model
learn model measurement prioriEstimate =
    let
        prioriCovariance =
            (model.params.stateFactor ^ 2 * model.state.covariance)
                + model.params.desiredNoisePower

        kalmanGain =
            prioriCovariance
                * model.params.measurementFactor
                / (((model.params.measurementFactor ^ 2) * prioriCovariance)
                    + model.params.expectedNoisePower
                  )
    in
        { model
            | state =
                { prediction =
                    prioriEstimate
                        + kalmanGain
                        * (measurement
                            - (model.params.measurementFactor * prioriEstimate)
                          )
                , covariance =
                    prioriCovariance
                        - (kalmanGain
                            * model.params.measurementFactor
                            * prioriCovariance
                          )
                }
        }


filterMaybeNoiseEstimates : Maybe Params -> List ( Float, Maybe Float ) -> List Float
filterMaybeNoiseEstimates optionalParams examples =
    let
        ( noisySignal, maybeNoiseEstimate ) =
            List.unzip examples

        initialModel =
            noisySignal
                |> List.head
                |> Maybe.withDefault 0
                |> fromMeasurement optionalParams
    in
        examples
            |> List.foldl
                (\( measurement, maybeNoiseEstimate ) ->
                    \( model, predictions ) ->
                        let
                            newModel =
                                filterMeasurement
                                    model
                                    maybeNoiseEstimate
                                    measurement
                        in
                            ( newModel
                            , newModel.state.prediction :: predictions
                            )
                )
                ( initialModel, [] )
            |> Tuple.second
            |> List.reverse
