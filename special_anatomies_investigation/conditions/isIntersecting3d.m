function b = isIntersecting3d(line1, line2, varargin)

tol = 1e-5;
if ~isempty(varargin)
    tol = varargin{1};
end

[d, ~, ~] = distanceLines3d(line1, line2);

if d < tol
    b = true;
else
    b = false;
end

end