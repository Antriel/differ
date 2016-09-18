package states;

import luxe.Vector;
import luxe.Input;
import luxe.Color;

import differ.Collision;
import differ.math.Vector in V;
import differ.shapes.*;
import differ.data.*;

class Rays extends luxe.States.State {

    var beam: Ray;
    var others: Array<Ray>;
    var colors: Array<Color>;
    var results: Results<RayIntersection>;

    var flip = false;

    override function onenter<T>(_:T) {

        others = [];
        colors = [
            new Color(),
            new Color().rgb(0x00e7b3),
            new Color().rgb(0x00b1dd),
            new Color().rgb(0xa0e300)
        ];

        Main.display('move the mouse around');

        beam = new Ray( new V(50,300), new V(600,400), false );

        for(i in 0 ... colors.length) {
            var sx = (i+1) * 120;
            var ray = new Ray( new V(sx, 100), new V(sx, 500), false );
            others.push(ray);
            Main.rays.push(ray);
        }

        Main.rays.push(beam);

            //allocate a precreated list of results, to avoid allocating every time
        results = new Results<RayIntersection>(colors.length);

    }

    override function onleave<T>(_:T) {

        beam = null;
        others = null;

    }

    override function onmousemove( e:MouseEvent ) {
        if(beam != null) {
            if(beam.infinite) {
                var end = new Vector(e.pos.x, e.pos.y);
                end.subtract_xyz(beam.start.x, beam.start.y);
                end.normalize();
                end.multiplyScalar(9999);
                beam.end.x = end.x;
                beam.end.y = end.y;
            } else {
                beam.end.x = e.pos.x;
                beam.end.y = e.pos.y;
            }
        }
    }

    override function onkeyup(e:KeyEvent) {
        
        if(e.keycode == Key.space) {
            if(beam != null) beam.infinite = !beam.infinite;
        }

        if(e.keycode == Key.key_f) {
            if(beam != null) {
                flip = !flip;
                if(flip) {
                    beam.start.x = 100+(colors.length * 120);
                } else {
                    beam.start.x = 50;
                }
            }
        }
    }

    override function update(dt:Float) {

        var colls = Collision.rayWithRays(beam, others, results);

        Luxe.draw.text({
            point_size:15,
            pos:new Vector(Luxe.screen.w - 10, 10),
            align: right,
            text: 'Hit ${colls.length} rays',
            immediate:true,
        });

        var bstart = new Vector(beam.start.x, beam.start.y);

        var textYval = 30;
        var idx = flip ? (colors.length-1) : 0;

        for (c in colls) {
            var rend = new Vector(c.ray2.end.x, c.ray2.end.y);
            var rstart = new Vector(c.ray2.start.x, c.ray2.start.y);

            Luxe.draw.line({
                p0: rstart,
                p1: rstart.clone().add_xyz( c.ray2.dir.x * c.u2, c.ray2.dir.y * c.u2 ),
                color: colors[idx],
                batcher: Main.thicker,
                immediate: true
            });

            Luxe.draw.line({
                p0: bstart,
                p1: bstart.clone().add_xyz( c.ray1.dir.x * c.u1, c.ray1.dir.y * c.u1 ),
                color: colors[idx],
                batcher: Main.thicker,
                depth: (others.length*2) - idx,
                immediate: true
            });

            Luxe.draw.text({
                point_size:13,
                pos:new Vector(Luxe.screen.w - 10,textYval),
                align: right,
                color: colors[idx],
                text: 'hit start %: ${c.u2}',
                immediate:true,
            });

            textYval += 30;
            if(flip) {
                idx--;
            } else {
                idx++;
            }
        }

    } //update

}
