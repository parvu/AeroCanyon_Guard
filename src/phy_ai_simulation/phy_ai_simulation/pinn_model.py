import torch
import torch.nn as nn

class PhysicsInformedDronePilot(nn.Module):
    def __init__(self, input_dim=6, output_dim=4, hidden_dim=64):
        super(PhysicsInformedDronePilot, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, output_dim)
        )
        
    def forward(self, state):
        return self.network(state)
