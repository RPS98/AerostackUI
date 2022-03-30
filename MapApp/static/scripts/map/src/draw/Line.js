class Line extends DrawManager {
    constructor(status, name, parameters = undefined, options = undefined, layerOptions = undefined) {
        super(status, 'Line', name, parameters, options, layerOptions);
    }

    codeDraw(values, options = undefined, layerOptions = {}, uavId = undefined) {
        if (values.length < 1) {
            return;
        }
        if (uavId !== undefined) {
            layerOptions['color'] = M.UAV_MANAGER.getColors(uavId)[1];
        }
        return super.codeDraw(values, options, layerOptions);
    }

    _addChangeCallback(id, info) {
        let htmlId = [];
        let nameId = [];
        let values = info.layer._latlngs;
        for (let i = 0; i < values.length; i++) {
            htmlId.push(`${id}-${i}-lat`);
            htmlId.push(`${id}-${i}-lng`);

            nameId.push(`${i}-lat`);
            nameId.push(`${i}-lng`);
        }
        Utils.addFormCallback(`${id}-change`, htmlId, nameId, this._changeCallback.bind(this), id, info);
    }

    _changeCallback(myargs, inputs) {
        let layer = myargs[1].layer;
        
        let values = [];
        let len = Math.floor(Object.keys(inputs).length / 2);
        for (let i = 0; i < len; i++) {
            values.push([]);
        }
        for (var key in inputs) {
            let value = inputs[key];
            let keyParse = key.split('-');
            let index1 = parseInt(keyParse[0]);
            if (keyParse[1] == 'lat') {
                values[index1][0] = value;
            } else if (keyParse[1] == 'lng') {
                values[index1][1] = value;
            }
        }

        layer.setLatLngs(values);
    }

    drawInfoAdd(htmlId, info, name = "Line", initialHtml = [], endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs;
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
            let lngDict = HTMLUtils.addDict('input', `${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' });

            initialHtml.push(row);
        }

        return super.drawInfoAdd(htmlId, info, name, initialHtml, endHtml, uavPickerType);
    }
}

class Path extends Line {
    constructor(status, options = undefined, layerOptions = undefined, parameters = config.Layers.Line.Path.parameters) {
        super(status, 'Path', parameters, options, layerOptions);
    }

    drawInfoAdd(id, info) {
        let name = 'Path';
        return super.drawInfoAdd(id, info, name, undefined, undefined, 'radio');
    }
}


class Odom extends Line {
    constructor(status, options = undefined, layerOptions = undefined, parameters = undefined) {
        super(status, 'Odom', parameters, options, layerOptions);
    }
}


class DesiredPath extends Line {
    constructor(status, options = undefined, layerOptions = undefined, parameters = undefined) {
        super(status, 'DesiredPath', parameters, options, layerOptions);
    }
}