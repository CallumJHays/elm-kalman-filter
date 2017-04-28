module Visualization exposing (main)

import Html exposing (..)
import Html.Attributes as HA exposing (..)
import Html.Events exposing (onClick, onInput)
import Random
import Random.Float
import EveryDict
import Charty.LineChart as LineChart
import Unwrap
import KalmanFilter


type alias Model =
    { noise : List Float
    , sliders : EveryDict.EveryDict SliderOption Slider
    , signalFunction : Float -> Float
    }


init : ( Model, Cmd Msg )
init =
    { noise = []
    , sliders = initSliders
    , signalFunction = (logBase 2)
    }
        |> regenerateNoise


getSliderVal : Model -> SliderOption -> Float
getSliderVal model sliderOption =
    model.sliders
        |> EveryDict.get sliderOption
        |> Unwrap.maybe
        |> .value


getXRange : Model -> ( Float, Float, Float )
getXRange model =
    let
        xRange =
            [ XRangeMin, XRangeMax, XRangeStep ]
                |> List.map (getSliderVal model)
    in
        case xRange of
            [ from, to, step ] ->
                ( from, to, step )

            _ ->
                Debug.crash "Could not get XRange"


type SliderOption
    = XRangeMin
    | XRangeMax
    | XRangeStep
    | NoisePower
    | ExpectedNoisePower
    | DesiredNoisePower
    | StateFactor
    | ControlFactor
    | MeasurementFactor


type alias Slider =
    { value : Float, minVal : Float, maxVal : Float, step : Float }


initSliders : EveryDict.EveryDict SliderOption Slider
initSliders =
    let
        defaultSlider =
            { value = 50, minVal = 0, maxVal = 100, step = 1 }
    in
        EveryDict.fromList
            [ ( XRangeMin, { defaultSlider | value = 1 } )
            , ( XRangeMax
              , { defaultSlider | value = 100, minVal = 100, maxVal = 5000 }
              )
            , ( XRangeStep
              , { defaultSlider | value = 5, minVal = 1, maxVal = 100, step = 0.1 }
              )
            , ( NoisePower
              , { defaultSlider | value = 1, minVal = 0, maxVal = 10, step = 0.1 }
              )
            , ( ExpectedNoisePower
              , { defaultSlider | value = 1, minVal = 0, maxVal = 100, step = 0.1 }
              )
            , ( DesiredNoisePower
              , { defaultSlider | value = 1, minVal = 0, maxVal = 1, step = 0.001 }
              )
            , ( StateFactor
              , { defaultSlider | value = 1, minVal = 0.9, maxVal = 1.1, step = 0.001 }
              )
            , ( ControlFactor
              , { defaultSlider | value = 1, minVal = 0, maxVal = 10, step = 0.01 }
              )
            , ( MeasurementFactor
              , { defaultSlider | value = 1, minVal = 0.5, maxVal = 1.5, step = 0.01 }
              )
            ]


type Msg
    = RegenerateNoise
    | NewNoise (List Float)
    | UpdateSlider SliderOption Float
    | ChangeFunction FunctType


type FunctType
    = Log2
    | Constant
    | Linear
    | Quadratic
    | Digital


applyFunc : FunctType -> Float -> Float
applyFunc funcType =
    case funcType of
        Log2 ->
            logBase 2

        Constant ->
            always 10

        Linear ->
            (\x -> 2.5 * x)

        Quadratic ->
            (\x -> 0.001 * (x - 10) ^ 2 - (x - 10))

        Digital ->
            (\x -> (round (x / 30)) % 2 |> toFloat)


update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        RegenerateNoise ->
            regenerateNoise model

        NewNoise noise ->
            { model | noise = noise } ! []

        UpdateSlider sliderOption value ->
            let
                model1 =
                    { model
                        | sliders =
                            model.sliders
                                |> EveryDict.update
                                    sliderOption
                                    (Maybe.map
                                        (\slider -> { slider | value = value })
                                    )
                    }

                ( model2, cmds ) =
                    case sliderOption of
                        XRangeMin ->
                            regenerateNoise model1

                        XRangeMax ->
                            regenerateNoise model1

                        XRangeStep ->
                            regenerateNoise model1

                        NoisePower ->
                            regenerateNoise model1

                        _ ->
                            model1 ! []
            in
                model2 ! [ cmds ]

        ChangeFunction funcType ->
            { model | signalFunction = applyFunc funcType } ! []


regenerateNoise : Model -> ( Model, Cmd Msg )
regenerateNoise model =
    let
        ( from, to, step ) =
            getXRange model

        noisePower =
            getSliderVal model NoisePower

        generator =
            (Random.list
                ((to - from) / step |> round)
                (Random.Float.normal 0 noisePower)
            )
    in
        model
            ! [ Random.generate NewNoise generator ]


view : Model -> Html Msg
view model =
    let
        ( from, to, step ) =
            getXRange model

        xAxis =
            range from to step

        signal =
            xAxis |> List.map model.signalFunction

        noisySignal =
            signal |> List.map2 (+) model.noise

        filterParams =
            { expectedNoisePower = getSliderVal model ExpectedNoisePower
            , desiredNoisePower = getSliderVal model DesiredNoisePower
            , stateFactor = getSliderVal model StateFactor
            , controlFactor = getSliderVal model ControlFactor
            , measurementFactor = getSliderVal model MeasurementFactor
            }

        filter =
            KalmanFilter.fromMeasurement
                (noisySignal |> (List.head >> Maybe.withDefault 0))
                (Just filterParams)

        predictedSignal =
            noisySignal
                |> List.foldl
                    (\measurement ->
                        \( filter, predictions ) ->
                            let
                                newFilter =
                                    KalmanFilter.filter
                                        filter
                                        Nothing
                                        measurement

                                prediction =
                                    Debug.log
                                        (newFilter.state.covariance |> toString)
                                        newFilter.state.prediction
                            in
                                ( newFilter, prediction :: predictions )
                    )
                    ( filter, [] )
                |> Tuple.second
                |> List.reverse

        lineChartConfigDefault =
            LineChart.defaults

        dataset =
            [ { label = "True Signal", data = List.map2 (,) xAxis signal }
            , { label = "Noisy Signal", data = List.map2 (,) xAxis noisySignal }
            , { label = "Predicted Signal", data = List.map2 (,) xAxis predictedSignal }
            ]

        signalFuncOptions =
            div []
                ([ Log2, Constant, Linear, Quadratic, Digital ]
                    |> List.map
                        (\funcType ->
                            div []
                                [ input
                                    [ type_ "radio"
                                    , name "signalFunc"
                                    , onClick (ChangeFunction funcType)
                                    ]
                                    []
                                , text (funcType |> toString)
                                ]
                        )
                )
    in
        div []
            [ div [ style [ ( "width", "800px" ) ] ]
                [ LineChart.view
                    { lineChartConfigDefault | drawPoints = False }
                    dataset
                ]
            , div [ style [ ( "display", "inline-block" ), ( "margin", "10px" ) ] ]
                [ h4 [] [ text "Signal type" ]
                , signalFuncOptions
                ]
            , div [ style [ ( "display", "inline-block" ), ( "margin", "10px" ) ] ]
                [ h4 [] [ text "Signal Controls" ]
                , button [ onClick RegenerateNoise ] [ text "Regenerate Noise" ]
                , viewSlider model XRangeMin
                , viewSlider model XRangeMax
                , viewSlider model XRangeStep
                , viewSlider model NoisePower
                ]
            , div [ style [ ( "display", "inline-block" ), ( "margin", "10px" ) ] ]
                [ h4 [] [ text "Kalman Filter Params" ]
                , viewSlider model ExpectedNoisePower
                , viewSlider model DesiredNoisePower
                , viewSlider model StateFactor
                , viewSlider model ControlFactor
                , viewSlider model MeasurementFactor
                ]
            ]


viewSlider : Model -> SliderOption -> Html Msg
viewSlider model sliderOption =
    let
        sliderModel =
            model.sliders
                |> EveryDict.get sliderOption
                |> Unwrap.maybe
    in
        div []
            [ label [] [ text ((sliderOption |> toString) ++ ": ") ]
            , br [] []
            , text ((sliderModel.minVal |> toString) ++ " <= ")
            , input
                [ type_ "range"
                , HA.min (sliderModel.minVal |> toString)
                , HA.max (sliderModel.maxVal |> toString)
                , value (sliderModel.value |> toString)
                , step (sliderModel.step |> toString)
                , onInput
                    (String.toFloat
                        >> Unwrap.result
                        >> UpdateSlider sliderOption
                    )
                ]
                []
            , text
                (" = "
                    ++ (sliderModel.value |> toString)
                    ++ " <= "
                    ++ (sliderModel.maxVal |> toString)
                )
            ]


range : Float -> Float -> Float -> List Float
range from to step =
    let
        newTo =
            round ((to - from) / step)
    in
        List.range 0 newTo |> List.map (\x -> (x |> toFloat) + from)


main : Program Never Model Msg
main =
    program
        { init = init
        , update = update
        , subscriptions = always Sub.none
        , view = view
        }
