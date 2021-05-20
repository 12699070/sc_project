function [centerLine_left, centerLine_right] = ThresholdGenerator(camWidth,camHeight,desiredCenterWidth)

centerLine_left = [camWidth/2-desiredCenterWidth camHeight camWidth/2-desiredCenterWidth 0];
centerLine_right = [camWidth/2+desiredCenterWidth camHeight camWidth/2+desiredCenterWidth 0];