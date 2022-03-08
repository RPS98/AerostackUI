class Line extends DrawManager
{
    constructor(name, options = undefined) {
        super('Line', name, options);
    }

    codeDraw(values, options={}) {
        if (values.length > 1) {
            return super.codeDraw(values, options);
        }
    }

    userDraw(options={}) {
        return super.userDraw(options);
    }
}

class Path extends Line
{
    constructor(options = undefined) {
        super('Path', options);
    }

    getHtmlDrawInfo(htmlId, layer) {

        let id = layer.layer.pm.options.DrawManager.idUserDraw;
        let values = layer.layer._latlngs;
        let rows = [];
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${htmlId}-${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', }, 'number', lat);
            let lngDict = HTMLUtils.addDict('input', `${htmlId}-${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', }, 'number', lng);
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], {'class': 'col-6'});

            rows.push(row);
        }

        return super.getHtmlDrawInfo(htmlId, layer, 'Path', rows)
    }
}


class Odom extends Line
{
    constructor(options = undefined) {
        super('Odom', options);
    }

    codeDraw(id, values, options={}) {
        options['color'] = M.UAV_MANAGER.getColors(id)[1];
        return super.codeDraw(values, options);
    }
}


class DesiredPath extends Line
{
    constructor(options = undefined) {
        super('DesiredPath', options);
    }

    codeDraw(id, values, options={}) {
        options['color'] = M.UAV_MANAGER.getColors(id)[1];
        options['opacity'] = 0.5;

        return super.codeDraw(values, options);
    }
}