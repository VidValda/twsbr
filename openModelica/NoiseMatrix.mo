block DriftingVectorNoise
  "Adds integrated normal distributed noise (random walk drift) to a vector signal of size n.
  This block is designed to model the 'Drift'."

  // --- Parameter for Vector Dimension ---
  parameter Integer n(min=1) = 1 "Dimension of the input/output vectors";

  // --- Connectors ---
  input Modelica.Blocks.Interfaces.RealInput u[n] "Uncorrupted input signal"
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  output Modelica.Blocks.Interfaces.RealOutput y[n] "Output signal with added drift (y = u + drift)"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));


  // --- Parameters for Noise Generation ---

  parameter Real sigma_d[n](each min=0, each start=1) = fill(1.0, n)
    "Vector of drift rate standard deviations. This corresponds to sigma_d in the report"
    annotation (Dialog(enable=enableNoise, group="Noise Parameters"));

  parameter Real samplePeriod(min=1e-6, start=0.01) = 0.01
    "Sample period (MUST be > 0 for discrete random walk model). Corresponds to Ts in the paper."
    annotation (Dialog(enable=enableNoise, group="Noise Parameters"));

  parameter Real startTime = 0 "Start time of sampling"
    annotation (Dialog(enable=enableNoise, group="Noise Parameters"));

  parameter Boolean enableNoise = true "Enable noise generation (if false, y = u)"
    annotation (Dialog(group="Noise Parameters"));


  // --- Internal State for Accumulated Drift ---
  discrete Real drift[n](each start=0, each fixed=true) "Accumulated drift state (the random walk)";


protected
  // --- Internal Noise Source for the *increment* ---
  // This generates the delta_d[k] term.
  // The mean (mu) is 0, as per the report presented N(0,1).
  // The standard deviation (sigma) is sigma_d * sqrt(samplePeriod),
  // as derived from the paper's equation for the increment.
  Modelica.Blocks.Noise.NormalNoise noiseIncrementSource[n](
    each mu = 0.0,
    sigma = sigma_d * sqrt(samplePeriod),
    each samplePeriod = samplePeriod,
    each startTime = startTime,
    each enableNoise = enableNoise);


initial equation
  // Start the drift at zero
  drift = fill(0.0, n);


equation
  // --- Discrete Update for Random Walk ---
  // This equation is only executed at the discrete sample instants.
  // It implements: drift[k] = drift[k-1] + noise_increment[k]
  when sample(startTime, samplePeriod) then
    drift = pre(drift) + noiseIncrementSource.y;
  end when;

  // --- Output Equation ---
  // Add the accumulated drift to the input signal.
  // If enableNoise is false, noiseIncrementSource.y will be 0,
  // so the drift will remain 0.
  y = u + drift;


  // --- Annotation for Icon and Documentation ---
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
    graphics={
      Rectangle(extent={{-100,-100},{100,100}}, fillPattern=FillPattern.Solid, fillColor={255,255,255}, lineColor={0,0,0}),
      Text(extent={{-80,40},{80,-40}}, horizontalAlignment=TextAlignment.Center, textString="u + ∫N(0,σ_d²)"),
      Text(extent={{-150, 15},{-110, -15}}, horizontalAlignment=TextAlignment.Right, textString="u[n]"),
      Text(extent={{110, 15},{150, -15}}, horizontalAlignment=TextAlignment.Left, textString="y[n]")
    }));
end DriftingVectorNoise;