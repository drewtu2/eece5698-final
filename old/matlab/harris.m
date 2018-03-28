function [varargout] = harris(I,varargin)
%HARRIS Harris corner detector
%  This function implements a version of the Harris corner detector which
%  has the ability to calculate the eigenvalues of the gradient matrix
%  directly.  This is opposed to calculating the corner response function as
%  originally proposed by Harris in:
%
%  C. Harris and M. Stephens.  "A Combined Corner and Edge
%  Detector", Proceedings of the 4th Alvey Vision Conference,
%  Manchester, U.K. pgs 147-151, 1988
%
%  INPUT:
%  I is the graylevel image to extract interest points from
%  PARAMETER     DEFAULT   DESCRIPTION
%  ----------   --------  --------------------------
%  N               500    maximum number of interest points to return
%  'disp'          N/A    overlays corner points on image
%  'subpixel'      N/A    analytically calculate subpixel precision of 
%                         corner locations by fitting a quadratic
%                         surface to corner response function
%  'thresh'          0    smallest acceptable value of response function
%  'hsize_i          3    size of the smoothing Gaussian mask
%  'sigma_i         0.5   standard deviation of Gaussian filter
%  'tile'          [1 1]  break the image into regions [y x] to
%                         distribute feature points more uniformly
%  'mask'           M     array of 1's same size as image defining
%                         where to compute feature points (useful
%                         for radially compensated images)
%  'eig'           N/A    use smallest eigenvalue as response
%                         function, o.w. use response function (default)
%                         originally proposed by Harris
%  'fft'           N/A    perform smoothing filtering in freqeuncey
%                         domain, o.w. perform in spatial domain (default)
%
%  OPTIONAL OUTPUT:
%  (y,x) are the row/column locations of interest points
%  M is the corner response function value associated with that point
%
%  Example:
%  I = imread('cameraman.tif');
%  [y,x,m] = harris(I,1000,'tile',[2 2],'disp');
%
%Date        Who     What
%--------    ----    ---------------------------------    
%20021010    rme     Create.
%20021014    rme     Added option of subpixel precision.
%20021110    rme     Added example and no ouput option
%20021214    rme     Added tiling and mask features as well as
%                    Harris corner response function. 
%20030821    rme     Speed up subpixel code, as well as regional
%                    processing code.
%20030826    rme     Added 'fft' option to speed up feature extraction
%                    for large images (e.g. 1024x1280)
%20041218    rme     Added convolve2.m function call.
%                    convolve2 performs SVD on the mask and then performs separable
%                    convolution.  note that matlab's filter2 also tries to do this, but for
%                    some reason calling conv2(h1,h2,I) is *much* slower than calling
%                    conv2(conv2(I,h1),h2) which what convolve2 does.
  
param = checkargs(size(I),varargin{:});

I = double(I);

[nr,nc] = size(I);

% create gradient masks
dx = [-1 0 1; -1 0 1; -1 0 1]/3;
dy = dx';

% calculate image gradients
Ix = convolve2(I,dx,'same');
Iy = convolve2(I,dy,'same');

% calculate gradient products
IxIx = Ix.*Ix;
IyIy = Iy.*Iy;
IxIy = Ix.*Iy;

% smooth squared image gradients
gmask = fspecial('gaussian',param.hsize,param.sigma);
if param.fft == false
  IxIx = convolve2(IxIx,gmask,'same');
  IyIy = convolve2(IyIy,gmask,'same');
  IxIy = convolve2(IxIy,gmask,'same');
else
  m = size(IxIx,1)+size(gmask,1)-1;
  n = size(IxIx,2)+size(gmask,2)-1;
  try
    % previous values
    pv_t = load(strcat(tempdir,'harris_prev_val.mat'));
    if m == pv_t.m && n == pv_t.n && ...
	  param.hsize == pv_t.hsize && param.sigma == pv_t.sigma
      % load previously stored smoothing filter fft
      %disp('loading gmask_fft');
      gmask_fft = pv_t.gmask_fft;
    else
      %disp('generating gmask_fft');
      gmask_fft = fft2(gmask,m,n);
    end
  catch
    %disp('no previous values exist, generating gmask_fft')
    hsize = param.hsize;
    sigma = param.sigma;
    % no previous values exist
    gmask_fft = fft2(gmask,m,n);
    save(strcat(tempdir,'harris_prev_val.mat'), ...
  	 'hsize','sigma','m','n','gmask_fft');
  end

  IxIx = real(ifft2(fft2(IxIx,m,n).*gmask_fft));
  IyIy = real(ifft2(fft2(IyIy,m,n).*gmask_fft));
  IxIy = real(ifft2(fft2(IxIy,m,n).*gmask_fft));
  % keep 'same' portion
  w = (param.hsize-1)/2; % hsize is assumed to be odd
  IxIx = IxIx(w+1:w+nr,w+1:w+nc);
  IyIy = IyIy(w+1:w+nr,w+1:w+nc);
  IxIy = IxIy(w+1:w+nr,w+1:w+nc);
  clear hsize sigma m n gmask_fft gmask;
end

% calculate the eigenvalues of the matrix
% [IxIx IxIy
%  IxIy IyIy]
% old way
%lambda1a = (IxIx+IyIy)/2 + 1/2*sqrt((IxIx+IyIy).^2-4*(IxIx.*IyIy-IxIy.*IxIy));
%lambda2a = (IxIx+IyIy)/2 - 1/2*sqrt((IxIx+IyIy).^2-4*(IxIx.*IyIy-IxIy.*IxIy));
% faster way
B = (IxIx+IyIy);
SQRTRM = sqrt(B.^2 - 4*(IxIx.*IyIy-IxIy.^2));
lambda1 = (B+SQRTRM)/2;
lambda2 = (B-SQRTRM)/2;
clear B SQRTRM IxIx IyIy IxIy;

% corner detection is based upon the desired corner response function
if param.eig
  % minimum eigenvalue
  R = real(min(lambda1,lambda2));
else
  % Harris corner response function
  % trace equals sum of eigenvalues
  % determinant equals product of eigenvalues
  R = lambda1.*lambda2 - 0.04*(lambda1+lambda2).^2;
end

% locate local maxima based upon eight-connected neighborhood
Maxima = (imregionalmax(R) & param.mask).*R;

% sort interest points by reponse function
% only consider pixels where the gradient matrix is positive definite
% i.e. both eigenvalues are greater than zero
[i,j] = find(Maxima > param.thresh);
m = Maxima((j-1)*nr+i); %shortcut for maxima(sub2ind(size(maxima),i,j));
[m,idx] = sort(m);
i = i(idx);
j = j(idx);
    
% interest points were sorted ascendingly by response function,
% flip so largest response points are first
m = m(end:-1:1);
i = i(end:-1:1);
j = j(end:-1:1);

if param.tile(1) > 1 && param.tile(2) > 1
  % process image regionally so that corners are uniformally
  % extracted across image regions
  ii = [];
  jj = [];
  mm = [];
  Npts_per_region = round(param.N/prod(param.tile));
  xx = round(linspace(1,nc,param.tile(2)+1)); % region boundaries
  yy = round(linspace(1,nr,param.tile(1)+1));
  for pp = 2:length(xx)
    % points falling within the region's x boundaries
    idx = find((j >= xx(pp-1)) & (j < xx(pp)));
    for qq = 2:length(yy)
      % points falling within the region's y boundaries
      idy = find((i >= yy(qq-1)) & (i < yy(qq)));
      
      % their common intersection
      ind = intersect(idx,idy);
      
      % return the strongest N points as defined by user
      ii = [ii; i(ind(1:min(end,Npts_per_region)))];
      jj = [jj; j(ind(1:min(end,Npts_per_region)))];
      mm = [mm; m(ind(1:min(end,Npts_per_region)))];
    end
  end
else
  ii = i(1:min(end,param.N));
  jj = j(1:min(end,param.N));
  mm = m(1:min(end,param.N));
end

if param.subpixel
  % refine corner locations with subpixel accuracy
  % fit a quadratic surface to maxima location
  % and calculate it's analytic peak
  [ii,jj] = subpixel(ii,jj,R);
end

if param.disp | nargout == 0
  % overlay corner points on original image
  figure;
  imagesc(I);
  colormap gray;
  hold on;
  plot(jj,ii,'y+');
  hold off;
  drawnow;
end

if nargout >= 2
  varargout{1} = ii;
  varargout{2} = jj;
end
if nargout == 3
  varargout{3} = mm;
end

%=======================================================================
function param = checkargs(isize,varargin);

% set defaults
param.disp = 0;      % overlay corner points on image
param.N = 500;       % number of interest points to return
param.subpixel = false;  % refine corner location with subpixel accuracy
param.thresh = 0;    % threshold value for smallest response magnitude
param.hsize = 3;     % size of gaussian smoothing mask
param.sigma = 0.5;   % standard deviation of gaussian mask
param.eig   = false; % use corner response function as originally
                     % proposed by Harris
param.tile = [1 1];  % do not process image regionally
param.fft  = false;  % implement conv2 in spatial domain

%----------------------------------------------
% replace defaults with user specified values
%----------------------------------------------
% check to see if number of corner points was specified
% otherwise use default
if nargin > 1 && isnumeric(varargin{1})
  param.N = varargin{1};
  ii = 2;
else
  ii = 1;
end

% loop through parameter/value pairs
while ii <= nargin - 1;
  switch lower(varargin{ii})
   case 'disp'
    param.disp = 1;
    ii = ii+1;
   case 'subpixel'
    param.subpixel = true;
    ii = ii+1;
   case 'eig'
    param.eig = true;
    ii = ii+1;
   case 'thresh'
    param.thresh = varargin{ii+1};
    ii = ii+2;
   case 'hsize'
    param.hsize = varargin{ii+1};
    ii = ii+2;
   case 'sigma'
    param.sigma = varargin{ii+1};
    ii = ii+2;
   case 'mask'
    param.mask  = logical(varargin{ii+1});
    ii = ii+2;
   case 'tile'
    param.tile  = varargin{ii+1};
    ii = ii+2;
   case 'fft'
    param.fft = true;
    ii = ii+1;
   otherwise
    error(sprintf('Uknown option ''%s''',varargin{ii}));
  end
end

if ~isfield(param,'mask')
  % corner mask defines region to compute interest points
  % define default mask to include "valid" smoothed 
  % image gradient portions from convolution
  param.mask = true(isize);
  %h = ceil((param.hsize-1)/2);
  %param.mask(1:h,:) = false;
  %param.mask(end-h-1:end,:) = false;
  %param.mask(:,1:h) = false;
  %param.mask(:,end-h-1:end) = false;
end

%================================================================
% the current implementation no longer uses this function
function mask = regionalmask(tile,msize,ii,jj)

x = 1:floor((msize(2)-1)/tile(2)):msize(2);
y = 1:floor((msize(1)-1)/tile(1)):msize(1);

mask = false(msize);
mask(y(jj):y(jj+1)-1,x(ii):x(ii+1)-1) = true;


%================================================================
function [isub,jsub] = subpixel(i,j,R)

% fit a quadratic surface to each integer maxima
% location defined by [i,j] using its surrounding
% eight neighbors.
%
% a*i^2 + b*i*j + c*j^2 + d*i + e*j+f = R(i,j)
%
% write above equation in matrix format
% [i^2 i*j j^2 i j 1][a b c d e f]' = R(i,j)
%
% ALGORITHM OUTLINE:
%------------------------------------------------------
% stack equations for all 9 pixels PER maxima
% i.e. A*pvec = B
% solve using least squares for one 6x1 parameter
% vector which defines a quadratic surface for maxima
% i.e. pvec = B\A where pvec = [a b c d e f]'
%------------------------------------------------------

% number of feature points
N = length(i);

% location of quadratic surface maximum as expressed
% in local coordinates
jmax = zeros(N,1);
imax = zeros(N,1);

% [p,q] are the local pixel coordinates centered around
% feature point [i,j]
p = [-1  0  1 -1 0 1 -1 0 1]';
q = [-1 -1 -1  0 0 0  1 1 1]';
A = [p.^2, p.*q, q.^2, p, q, ones(9,1)];
B = zeros(9,1);

for n = 1:N
  % calculate measurement vector
  for k=1:9
    B(k) = R(i(n)+p(k),j(n)+q(k));
  end
  
  % calculate least-squares parameter vector
  pvec = B\A;
  a = pvec(1); b = pvec(2); c = pvec(3);
  d = pvec(4); e = pvec(5); f = pvec(6);
  
  % analytically calculate location of surface maxima
  % by solving the two equations listed below
  % d/di = 0 = 2*a*i + b*j + d
  % d/dj = 0 = b*i + 2*c*j + e
  jmax(n) = (2*a*e-d*b)/(b^2-4*a*c);
  imax(n) = -(2*c*jmax(n)+e)/b;
end

% update maxima locations with subpixel precision
isub = i+imax;
jsub = j+jmax;
