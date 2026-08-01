import torch
import torch.nn as nn
import torch.optim as optim

class PhysicsInformedDronePilot(nn.Module):
    def __init__(self, input_dim=6, output_dim=4, hidden_dim=64):
        super(PhysicsInformedDronePilot, self).__init__()
        # Input layer: 3 IMU Accel + 3 IMU Gyro
        self.network = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, output_dim) # Output: 4 motor thrust values
        )
        
    def forward(self, state):
        return self.network(state)

def physics_informed_loss(pred_thrust, target_thrust, state, mass=1.5, g=9.81):
    # Data-driven tracking loss (Standard MSE)
    loss_data = nn.MSELoss()(pred_thrust, target_thrust)
    
    # Physics constraint: Net thrust must balance gravity + aerodynamic drag residual
    # state[:, 2] is linear acceleration on Z axis
    accel_z = state[:, 2]
    total_pred_thrust = torch.sum(pred_thrust, dim=1)
    
    # Newton's 2nd Law constraint: F_thrust - m*g = m*a
    expected_force = mass * (accel_z + g)
    loss_physics = nn.MSELoss()(total_pred_thrust, expected_force)
    
    # Combined Loss
    return loss_data + 0.1 * loss_physics

if __name__ == "__main__":
    model = PhysicsInformedDronePilot()
    print("PINN Network Architecture initialized successfully:")
    print(model)
    
    # Dummy step verifying tracking execution
    sample_state = torch.randn(10, 6) # Batch of 10 telemetry frames
    sample_target = torch.rand(10, 4)
    prediction = model(sample_state)
    loss = physics_informed_loss(prediction, sample_target, sample_state)
    print(f"Sample initial loss computed: {loss.item():.4f}")
