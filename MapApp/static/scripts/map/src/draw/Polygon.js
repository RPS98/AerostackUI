class Polygon extends DrawManager
{
    constructor(name, options = undefined) {
        super('Polygon', name, options);
    }

    getHtmlDrawInfo(htmlId, layer, name="Area", htmlCode = undefined, addUavPicker = true, uavPickerType = 'checkbox') {

        let id = layer.pm.options.DrawManager.idUserDraw;

        console.log(`${id}`);
        console.log(layer);
        let values = layer._latlngs[0];

        let htmlValues = [];
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${htmlId}-${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', }, 'number', lat);
            let lngDict = HTMLUtils.addDict('input', `${htmlId}-${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', }, 'number', lng);
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], {'class': 'col-md-6'});

            htmlValues.push(row);
        }

        return super.getHtmlDrawInfo(htmlId, layer, name, htmlValues, htmlCode, addUavPicker, uavPickerType);
    }
}

class Area extends Polygon
{
    constructor(options = undefined) {
        super('Area', options);
    }

    getHtmlDrawInfo(htmlId, layer) {
        let name = 'Area';
        return super.getHtmlDrawInfo(htmlId, layer, name);
    }
}

