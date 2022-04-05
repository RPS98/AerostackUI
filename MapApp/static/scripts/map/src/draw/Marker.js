/**
 * Extends the Draw Manager to enable Marker drawing.
 */
class Marker extends DrawManager {
   /**
    * Creates a new Marker Draw Manager.
    * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
    * @param {string} name - Name of the layer, for example: 'PointOffInterest', 'LandPoint', 'UAVMarker'.
    * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
    * @param {dict} options - Options of the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {svg} svgConfig - Icon to use for the marker, and can change it color. Optional.
    */
   constructor(status, name, parameters = undefined, options = undefined, layerOptions = {}, svgConfig) {
      Marker._getIcon(svgConfig, layerOptions);

      super(status, 'Marker', name, parameters, options, layerOptions);
      this.svgConfig = svgConfig;
   }

   static _getIcon(svgConfig, layerOptions) {
      if (layerOptions.svg !== undefined) {
         svgConfig.svg = layerOptions.svg;
      }
      if (layerOptions.baseColor !== undefined) {
         svgConfig.baseColor = layerOptions.baseColor;
      }
      if (layerOptions.iconSize !== undefined) {
         console.log("Change icon size " + layerOptions.iconSize);
         svgConfig.iconSize = layerOptions.iconSize;
      }
      if (layerOptions.iconAnchor !== undefined) {
         console.log("Change icon anchor " + layerOptions.iconAnchor);
         svgConfig.iconAnchor = layerOptions.iconAnchor;
      }

      let desiredColor = Utils.deepCopy(svgConfig.baseColor);
      if (layerOptions.fillColor !== undefined) {
         desiredColor[0] = layerOptions.fillColor;
      }
      if (layerOptions.borderColor !== undefined) {
         desiredColor[1] = layerOptions.borderColor;
      }

      console.log("Change icon color " + desiredColor);

      let icon = Marker._changeIconColor(svgConfig.svg, desiredColor, svgConfig.baseColor, svgConfig.iconSize, svgConfig.iconAnchor);

      layerOptions['markerStyle'] = { 'icon': icon };
      layerOptions['icon'] = icon;
   }

   // #region Public methods

   /**
    * Extends the Draw Manager codeDraw to assign color to the Marker by the UAV id.
    * @param {L.latlng} values - Leaflet latitude and longitude of the layer (Reference: https://leafletjs.com/).
    * @param {dict} options - Extra options to add to the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {string} uavId - UAV id. Optional.
    * @param {svg} iconSvgBase - Icon to use for the marker, and can change it color. Optional.
    * @param {list} iconSize - List with the width and height of the icon. Optional.
    * @param {list} iconAnchor - List with the x and y offset of the icon. Optional.
    * @returns {L.Layer} - Instance of the layer created (Reference: https://leafletjs.com/).
    * @public
    */
   codeDraw(values, options = undefined, layerOptions = {}, desiredColor = undefined, svg = this.svgConfig) {
      if (desiredColor !== undefined) {
         layerOptions.fillColor = desiredColor[0];
         layerOptions.borderColor = desiredColor[1];

         Marker._getIcon(this.svgConfig, layerOptions);
      }
      super.codeDraw(values, options, layerOptions);
   }

   /**
    * Extends the drawInfoAdd of DrawManager to add Markers coordinates.
    * @param {string} htmlId - Base id of the HTML element to add.
    * @param {dict} info - Dict with the layer and the Draw Manager options.
    * @param {string} name - Name of the layer.
    * @param {list} initialHtml - List with the HTML to add at the beginning of the info.
    * @param {list} endHtml - List with the HTML to add at the end of the info.
    * @param {string} uavPickerType - Type of the UAV picker, for example 'checkbox' or 'radio'.
    * @returns {void}
    * @public
    */
   drawInfoAdd(htmlId, info, name = info.drawManager.options.name, initialHtml = [], endHtml = undefined, uavPickerType = 'radio') {

      let lat = info.layer._latlng.lat;
      let lng = info.layer._latlng.lng;

      let id = htmlId + '-' + info.id;
      let latDict = HTMLUtils.addDict('input', `${id}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
      let lngDict = HTMLUtils.addDict('input', `${id}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
      initialHtml.push(HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' }));

      return super.drawInfoAdd(htmlId, info, name, initialHtml, endHtml, uavPickerType);
   }

   /**
    * Modifies the Marker color.
    * @param {svg} svg
    * @param {list} desiredColor
    * @param {list} baseColor
    * @param {list} iconSize - List with the width and height of the icon. Optional.
    * @param {list} iconAnchor - List with the x and y offset of the icon. Optional.
    * @returns {L.DivIcon} - Icon with the new color (Reference: https://leafletjs.com/).
    * @public
    */
   static _changeIconColor(svg, desiredColor, baseColor, iconSize, iconAnchor) {

      let iconSvgModified = svg.replace(new RegExp(baseColor[0], 'g'), desiredColor[0]).replace(new RegExp(baseColor[1], 'g'), desiredColor[1]);

      return L.divIcon({
         html: iconSvgModified,
         className: "svg-icon",
         iconSize: iconSize,
         iconAnchor: iconAnchor,
      });
   }

   // #endregion

   // #region Private methods

   /**
    * Overload the Draw Manager _addChangeCallback manage change coordinates of the Marker.
    * @param {string} id - Base id of the HTML element to add.
    * @param {dict} info - Dict with the layer and the Draw Manager options.
    * @returns {void}
    * @private
    */
   _addChangeCallback(id, info) {
      Utils.addFormCallback(`${id}-change`, [`${id}-lat`, `${id}-lng`], ['lat', 'lng'], this._changeCallback.bind(this), info);
   }

   /**
    * Overload the Draw Manager _changeCallback to change the coordinates of the marker.
    * @param {list} myargs - List with the info dict, that has the layer and the Draw Manager options.
    * @param {dict} args - Dict with ['${id}-lat', '${id}-lng'] as keys and their values.
    * @returns {void}
    * @private
    */
   _changeCallback(myargs, inputs) {
      let layer = myargs[0].layer;
      layer.setLatLng(L.latLng(inputs.lat, inputs.lng));
   }

   // #endregion
}

/**
 * Type of Marker, use to create a Point of Interest.
 */
class PointOfInterest extends Marker {
   /**
    * Creates a new Point of Interest Marker Draw Manager.
    * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
    * @param {dict} options - Options of the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
    */
   constructor(status, options = undefined, layerOptions = undefined, parameters = config.Layers.Marker.PointOfInterest.parameters, svgConfig = config.Markers.PointOfInterest) {
      super(status, 'PointOfInterest', parameters, options, layerOptions, svgConfig);
   }
}

/**
 * Type of Marker, use to create a Waypoint.
 */
 class WayPoint extends Marker {
   /**
    * Creates a new WayPoint Marker Draw Manager.
    * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
    * @param {dict} options - Options of the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
    */
   constructor(status, options = undefined, layerOptions = undefined, parameters = config.Layers.Marker.WayPoint.parameters, svgConfig = config.Markers.WayPoint) {
      super(status, 'WayPoint', parameters, options, layerOptions, svgConfig);
   }
}

/**
 * Type of Marker, use to create a Land Point.
 */
 class LandPoint extends Marker {
   /**
    * Creates a new LandPoint Marker Draw Manager.
    * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
    * @param {dict} options - Options of the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
    */
   constructor(status, options = undefined, layerOptions = undefined, parameters = config.Layers.Marker.LandPoint.parameters, svgConfig = config.Markers.LandPoint) {
      super(status, 'LandPoint', parameters, options, layerOptions, svgConfig);
   }
}

/**
 * Type of Marker, use to create a Take Off Point.
 */
 class TakeOffPoint extends Marker {
   /**
    * Creates a new TakeOffPoint Marker Draw Manager.
    * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
    * @param {dict} options - Options of the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
    */
   constructor(status, options = undefined, layerOptions = undefined, parameters = config.Layers.Marker.TakeOffPoint.parameters, svgConfig = config.Markers.TakeOffPoint) {
      super(status, 'TakeOffPoint', parameters, options, layerOptions, svgConfig);
   }
}

/**
 * Type of Marker, use to create a UAV.
 */
 class UAVMarker extends Marker {
   /**
    * Creates a new UAVMarker Marker Draw Manager.
    * @param {string} status - Status of the layer, for example: 'draw', 'confirmed', 'uav'.
    * @param {dict} options - Options of the Draw Manager. Optional.
    * @param {dict} layerOptions - Options of the layer with Leaflet and PM options. Optional.
    * @param {list} parameters - List of parameters to add to options. Each parameter is a list of [type, name, value, text to add in input button]. Optional.
    */
   constructor(status, options = undefined, layerOptions = undefined, parameters = undefined, svgConfig = config.Markers.UAVMarker) {
      super(status, 'UAVMarker', parameters, options, layerOptions, svgConfig);
   }
}
