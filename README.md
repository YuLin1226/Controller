# Controller
These controllers are based on state feedback control, so they are model-based controller.
To apply these kind of controller, system identification is needed.

## System Identification
There are several ways to get a control model, here are ways I've tried:

### Grey Box
Construct the dynamic equation of the control model, and apply the parameters get from the experiment.

### Black Box
If it is hard to do the experiment to measure parameters for the model, another way is to try black box system id.
In black box, it is important to "guess" how many poles and zeros in your model.
Next step is "id" :
- Sine sweep
- Step response
