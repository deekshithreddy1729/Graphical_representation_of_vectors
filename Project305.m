function varargout = Project305(varargin)
% PROJECT305 MATLAB code for Project305.fig
%      PROJECT305, by itself, creates a new PROJECT305 or raises the existing
%      singleton*.
%
%      H = PROJECT305 returns the handle to a new PROJECT305 or the handle to
%      the existing singleton*.
%
%      PROJECT305('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECT305.M with the given input arguments.
%
%      PROJECT305('Property','Value',...) creates a new PROJECT305 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Project305_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Project305_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Project305

% Last Modified by GUIDE v2.5 15-Nov-2019 19:40:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Project305_OpeningFcn, ...
                   'gui_OutputFcn',  @Project305_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Project305 is made visible.
function Project305_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Project305 (see VARARGIN)

% Choose default command line output for Project305
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Project305 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Project305_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
k1=str2double(get(handles.k1,'string'));
k2=str2double(get(handles.k2,'string'));
k3=str2double(get(handles.k3,'string'));

k2=mod(k2,360);
k3=mod(k3,360);

if (k1<0)
    k1=-k1;
end

if (k2>180)
    k2=360-k2;
end

if (k3>180)
    k3=k3-360;
end

k2=k2*pi/180;
k3=k3*pi/180;

[x1,y1,z1] = sph2cart(k3,pi/2-k2,k1);

k4=str2double(get(handles.k4,'string'));
k5=str2double(get(handles.k5,'string'));
k6=str2double(get(handles.k6,'string'));

k5=mod(k5,360);
k6=mod(k6,360);

if (k4<0)
    k4=-k4;
end

if (k5>180)
    k5=360-k5;
end

if (k6>180)
    k6=k6-360;
end

k5=k5*pi/180;
k6=k6*pi/180;

[x2,y2,z2] = sph2cart(k6,pi/2-k5,k4);

A = [x1 y1 z1];
B = [x2 y2 z2];

C = cross(A,B);

x=C(1);
y=C(2);
z=C(3);

[phi,ele,rad] = cart2sph(x,y,z);

thetha=pi/2-ele;

if phi<0
   phi=phi+2*pi;
end

thetha=thetha*180/pi;
phi=phi*180/pi;

set(handles.rad,'string',rad)
set(handles.thetha,'string',thetha)
set(handles.phi,'string',phi)

cla
hold on

%quiver3(0, 0, 0, x1, y1, z1, 'Color', [0, 1, 0], 'AutoScale', 'off', 'LineWidth', 2);
%quiver3(0, 0, 0, x2, y2, z2, 'Color', [1, 0, 1], 'AutoScale', 'off', 'LineWidth', 2);

quiver3(0, 0, 0, x, y, z, 'Color', [0, 0.5, 0.8], 'AutoScale', 'off', 'LineWidth', 2);
quiver3(0, 0, 0, x, y, 0, 'Color', [0.8, 0.2, 0], 'AutoScale', 'off', 'LineWidth', 2);
plot3([x x], [y y], [0 z], 'r.:')
view(145, 45)

quiver3([0 0], [0 0], [0 0], [-rad rad], [0 0], [0 0], 'Color', [0, 0, 0], 'AutoScale', 'off')
quiver3([0 0], [0 0], [0 0], [0 0], [-rad rad], [0 0], 'Color', [0, 0, 0], 'AutoScale', 'off')
quiver3([0 0], [0 0], [0 0], [0 0], [0 0], [-rad rad], 'Color', [0, 0, 0], 'AutoScale', 'off')
text(rad/3,0,0, sprintf("X-axis"), "FontSize", 7);
text(0,rad/3,0, sprintf("Y-axis"), "FontSize", 7);
text(0,0,rad/3, sprintf("Z-axis"), "FontSize", 7);

r = rad/5;

phi_ = 0:1:phi;
x_phi = r * cosd(phi_);
y_phi = r * sind(phi_);
z_phi = zeros(1, length(phi_));
plot3(x_phi, y_phi, z_phi, 'Color', [0.8, 0.2, 0])
text(r, r, 0, sprintf("\\phi : %f",phi), "FontSize", 9);

thetha_ = 0:1:thetha;
x_thetha = r * sind(thetha_) * cosd(phi);
y_thetha = r * sind(thetha_) * sind(phi);
z_thetha = r * cosd(thetha_);
plot3(x_thetha, y_thetha, z_thetha, 'Color', [0, 0.5, 0.8])
text(r * cosd(phi), r * sind(phi), r, sprintf("\\theta : %f",thetha), "FontSize", 9);

plot3([0 x], [0 y], [-0.03*rad z-0.03*rad], 'b.:')
text(x/2,y/2,z/2-0.04*rad, sprintf("R : %f",rad), "FontSize", 9);

grid on
axis('equal')
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
title('Vector Plot')



function k1_Callback(hObject, eventdata, handles)
% hObject    handle to k1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k1 as text
%        str2double(get(hObject,'String')) returns contents of k1 as a double


% --- Executes during object creation, after setting all properties.
function k1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function k2_Callback(hObject, eventdata, handles)
% hObject    handle to k2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k2 as text
%        str2double(get(hObject,'String')) returns contents of k2 as a double


% --- Executes during object creation, after setting all properties.
function k2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function k3_Callback(hObject, eventdata, handles)
% hObject    handle to k3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k3 as text
%        str2double(get(hObject,'String')) returns contents of k3 as a double


% --- Executes during object creation, after setting all properties.
function k3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function k4_Callback(hObject, eventdata, handles)
% hObject    handle to k4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k4 as text
%        str2double(get(hObject,'String')) returns contents of k4 as a double


% --- Executes during object creation, after setting all properties.
function k4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function k5_Callback(hObject, eventdata, handles)
% hObject    handle to k5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k5 as text
%        str2double(get(hObject,'String')) returns contents of k5 as a double


% --- Executes during object creation, after setting all properties.
function k5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function k6_Callback(hObject, eventdata, handles)
% hObject    handle to k6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k6 as text
%        str2double(get(hObject,'String')) returns contents of k6 as a double


% --- Executes during object creation, after setting all properties.
function k6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rad_Callback(hObject, eventdata, handles)
% hObject    handle to rad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rad as text
%        str2double(get(hObject,'String')) returns contents of rad as a double


% --- Executes during object creation, after setting all properties.
function rad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thetha_Callback(hObject, eventdata, handles)
% hObject    handle to thetha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thetha as text
%        str2double(get(hObject,'String')) returns contents of thetha as a double


% --- Executes during object creation, after setting all properties.
function thetha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thetha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function phi_Callback(hObject, eventdata, handles)
% hObject    handle to phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phi as text
%        str2double(get(hObject,'String')) returns contents of phi as a double


% --- Executes during object creation, after setting all properties.
function phi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
