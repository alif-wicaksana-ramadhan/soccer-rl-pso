import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np


class FullyConnectedModel(nn.Module):
    def __init__(self, input_size, output_size):
        super(FullyConnectedModel, self).__init__()

        # Define layers with ReLU activation
        self.linear1 = nn.Linear(input_size, 16)
        self.activation1 = nn.ReLU()
        self.linear2 = nn.Linear(16, 16)
        self.activation2 = nn.ReLU()
        self.linear3 = nn.Linear(16, 16)
        self.activation3 = nn.ReLU()

        # Output layer without activation function
        self.output_layer = nn.Linear(16, output_size)

        # Initialization using Xavier uniform (a popular technique for initializing weights in NNs)
        nn.init.xavier_uniform_(self.linear1.weight)
        nn.init.xavier_uniform_(self.linear2.weight)
        nn.init.xavier_uniform_(self.linear3.weight)
        nn.init.xavier_uniform_(self.output_layer.weight)

    def forward(self, inputs):
        # Forward pass through the layers
        x = self.activation1(self.linear1(inputs))
        x = self.activation2(self.linear2(x))
        x = self.activation3(self.linear3(x))
        x = self.output_layer(x)
        return x
    

class SoccerObserverModel(nn.Module):
    def __init__(self, field_size=np.array([9.0, 6.0], np.float32)):
        super(SoccerObserverModel, self).__init__()
        self.field_size = field_size
        self.robot_pos_fc = nn.Linear(2, 64)
        self.robot_vel_fc = nn.Linear(2, 64)
        self.goal_fc = nn.Linear(2, 64)
        self.enemies_fc = nn.Linear(2, 64)

        self.aggregation_fc = nn.Linear(64, 64)

        self.fc1 = nn.Linear(4 * 64, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, 2)

    def forward(self, robot_position, robot_velocity, goal_position, enemies_positions):
        robot_position = torch.from_numpy(robot_position/self.field_size)
        robot_velocity = torch.from_numpy(robot_velocity/self.field_size)
        goal_position = torch.from_numpy(goal_position/self.field_size)
        enemies_positions = [torch.from_numpy(enemy_pos/self.field_size) for enemy_pos in enemies_positions]

        robot_pos_features = F.relu(self.robot_pos_fc(robot_position))
        robot_vel_features = F.relu(self.robot_vel_fc(robot_velocity))
        goal_features = F.relu(self.goal_fc(goal_position))
        enemies_features = [F.relu(self.enemies_fc(enemy_position)) for enemy_position in enemies_positions]

        agregated_enemies_features = sum(enemies_features)
        agregated_enemies_features = F.relu(self.aggregation_fc(agregated_enemies_features))

        combined_features = torch.cat((robot_pos_features, robot_vel_features, goal_features, agregated_enemies_features), dim=-1)

        x = F.relu(self.fc1(combined_features))
        x = F.relu(self.fc2(x))
        action = self.fc3(x)

        return action
    

class QNetwork:
    def __init__(self, lr, logdir=None):
        # Define Q-network with specified architecture
        # self.net = FullyConnectedModel(8, 2)
        self.net = SoccerObserverModel()
        self.lr = lr
        self.logdir = logdir
        self.optimizer = optim.Adam(self.net.parameters(), lr=self.lr)

    def load_model(self, model_file):
        # Load pre-trained model from a file
        return self.net.load_state_dict(torch.load(model_file))

    def load_model_weights(self, weight_file):
        # Load pre-trained model weights from a file
        return self.net.load_state_dict(torch.load(weight_file))
