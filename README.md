[//]: # (Image References)
 
[image1]: ./images/image_10.png
[image2]: ./images/image_11.png
[image3]: ./images/image_12.png
[image4]: ./images/image_1.png
[image5]: ./images/image_2.png
[image6]: ./images/image_3.png
[image7]: ./images/image_13.png

### Eduardo Ribeiro de Campos - April/2021.

# SFND-Radar-Target-Generation-and-Detection
#### UDACITY - Sensor fusion Nano degree - Radar course.


## Overview


Throughout the Radar course, we learned perspectives about Radar with [Andrei Vatavu](https://www.linkedin.com/in/vatavua).(Staff Software Engineer, Sensor Fusion) professional from [MBRDNA](https://www.mbrdna.com/) (Mercedes-Benz Reasearch & Development North America, Inc) team. and [Abdullah Zaidi](https://www.linkedin.com/in/abdullahzaidi) (Tech Lead, Level 5, Self-Driving Division). professional from [Lyft](https://self-driving.lyft.com/).

In this project lets Engineering a Collision Detection System, 

## Project Layout

![alt text |width=450px | align="middle"][image1]

## Goals

The mains tasks it is available in [rubric](https://review.udacity.com/#!/rubrics/2548/view) file.


## 1- Radar  Design Specifications.

![alt text |width=450px | align="middle"][image2].


The Design of the FMCW waveform by giving specs was performed on the file [Radar_Target_Generation_and_Detection.m
](./FinalProject_Camera.cpp) from lines 34 to 38.

```Matlab
SweeptTimeFactor = 5.5; % for FMCW must be considered 5 to 6 times the round trip time.

B = c /(2*Range_Resolution);
Tchirp = SweeptTimeFactor *( 2* Max_Range/c);
slope = B/Tchirp ;
 
```
The Calculated slop is: 2.04e13.

## 2- Moving target generation.

Implemented on the file [Radar_Target_Generation_and_Detection.m
](./FinalProject_Camera.cpp) from lines 24 to 25.

```Matlab
 initial_range = 110 ; %must be less than 200m
 velocity = 30; %must be in the range -70 to 70 m/s.
 
```

## 3- Signal Propagation.

Implemented on the file [Radar_Target_Generation_and_Detection.m
](./FinalProject_Camera.cpp) from lines 56 to 87.

![alt text |width=450px | align="middle"][image3].

```Matlab
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity.
    r_t(i) = initial_range + velocity*t(i);
    td(i) = (2 * r_t(i))/c;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + slope*(t(i)^2)/2));
    Rx (i) = cos(2*pi*( fc*(t(i) - td(i)) + slope*((t(i) - td(i))^2)/2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end
 
```

## 4- Processing Received reflected Signal.

Implemented on the file [Radar_Target_Generation_and_Detection.m
](./FinalProject_Camera.cpp) from lines 96 to 133.


```Matlab
Mix = reshape(Mix,[Nr,Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.

L = B*Tchirp;
Y = fft(Mix,Nr);

 % *%TODO* :
% Take the absolute value of FFT output

P2 = abs(Y/L);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.

P1 = P2(1:L/2+1);

%plotting the range
%figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 

%figure('Name','FFT Output')
fb = B*(0:(L/2))/L;
plot(fb,P1)
title('FFT Output')
%figure('Name','Range FFT1')
subplot(2,1,2)
Range = (c*Tchirp*fb)/(2*B);
plot(Range,P1)
title('Radar Range - FFT 1')

axis ([0 200 0 1]);
```
The next image shows the results.

![alt text |width=450px | align="middle"][image4].

it is possible identify the correct range calculated cosidering the input initial_range = 110m.


## 5- Range/Doppler FFT.

Implemented on the file [Radar_Target_Generation_and_Detection.m
](./FinalProject_Camera.cpp) from lines 147 to 162.


```Matlab
Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);
```

The next image shows the results.

![alt text |width=450px | align="middle"][image5].

## 6- CFAR Detection.

Implemented on the file [Radar_Target_Generation_and_Detection.m
](./FinalProject_Camera.cpp) from lines 171 to 261.

#### 6.1 - Determine the number of Training cells for each dimension. Similarly, pick the number of guard cells.

```Matlab
T_row = 10;
T_col = 5;

G_row = 3;
G_col = 1;

```

![alt text |width=450px | align="middle"][image7]


#### 6.2 - Slide the cell under test across the complete matrix. Make sure the CUT has margin for Training and Guard cells from the edges.
#### 6.3 - For every iteration sum the signal level within all the training cells. To sum convert the value from logarithmic to linear using db2pow function.

```Matlab
start_row = T_row + G_row + 1;
end_row = (Nr/2)-(T_row + G_row);
   
start_column = T_col + G_col + 1;
end_column = (Nd) - (T_col + G_col);

Training_cells_Size = (T_row*T_col)-(G_row*G_col);

   
for  it1 = start_row : end_row
    for it2 = start_column : end_column
        
        %iterate through training cells
        for it3 = it1 - T_row - G_row : it1 + T_row + G_row
            for it4 = it2- T_col - G_col : it2 + T_col + G_col               
               %checking position if are outside of the guardian cells
               if  ((it3 < G_row || it3 > G_row)&&(it4 < G_col || it4 > G_col))
                   noise_level = noise_level + db2pow(RDM(it3,it4));
               end
            end
        end

```

#### 6.4 - Average the summed values for all of the training cells used. After averaging convert it back to logarithmic using pow2db.

```Matlab
noise_level = noise_level/Training_cells_Size;   
        
threshold = pow2db(noise_level);

```

#### 6.5 - Further add the offset to it to determine the threshold.

```Matlab
 threshold =  threshold + offset;

```

#### 6.6 - Next, compare the signal under CUT against this threshold.
#### 6.7 - If the CUT level > threshold assign it a value of 1, else equate it to 0.

```Matlab
if (CUT > threshold)
    RDM(it1 , it2) = 1;
else
    RDM(it1 , it2) = 0;
end

```

#### 6.8 - To keep the map size same as it was before CFAR, equate all the non-thresholded cells to 0.

```Matlab
% For Rows
RDM(union(1:(T_row+G_row),end-(T_row + G_row - 1):end),:) = 0; 

% For Columns
RDM(:,union(1:(T_col + G_col),end-(T_col + G_col - 1):end)) = 0; 

```


The next image shows the results for the step 6 implementation.

![alt text |width=450px | align="middle"][image6].
