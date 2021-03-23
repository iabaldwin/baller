from matplotlib import cm

colormap = cm.seismic

with open('colormap.txt', 'w') as handle:
    for i in range(0, 256):
        r,g,b,a = colormap(i)
        handle.write('{} {} {}\n'.format(r, g, b))
