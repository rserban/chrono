file1 = 'rms1_normalized.obj';
file2 = 'rms1_normalized_mod.obj';

obj1 = read_wobj(file1);

v1 = obj1.vertices;

nv = length(v1);
dx = 0.0762;
z = v1(1:3:end,3);
nx = length(z);
x = dx * (0:nx-1)';
y_max = 4.0;
ny = 4;
dy = y_max / ny;
y = dy * (-ny:+ny);
ny = 2*ny+1;

v2 = zeros(nx * ny,3);
f2 = zeros(2*(nx-1)*(ny-1),3);

vv = 1;
ff = 1;
for i = 1:nx
   for j = 1:ny
       v2(vv,:) = [x(i) y(j) z(i)];
       vv = vv + 1;
   end
   if i < nx
      for j = 1:ny-1
          f2(ff,:) = [(i-1)*ny+j, i*ny+j, i*ny+j+1];
          f2(ff+1,:) = [(i-1)*ny+j, i*ny+j+1, (i-1)*ny+j+1];
          ff = ff + 2;
      end
   end
end

obj2.vertices = v2;
obj2.objects(1).type='f';
obj2.objects(1).data.vertices=f2;
write_wobj(obj2,file2);

