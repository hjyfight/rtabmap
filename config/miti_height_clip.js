/* Display-only point-cloud clipping. Coordinates and ROS messages remain untouched. */
function rtabmapHeightClipFactory(React) {
  'use strict';
  const h = React.createElement;
  const defaults = Object.freeze({enabled: false, height: 2});
  const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const Context = React.createContext({enabled: false, plane: [0, 0, 1, -2]});

  function normalize(value) {
    return {
      enabled: value?.enabled === true,
      height: typeof value?.height === 'number' && Number.isFinite(value.height)
        ? Math.max(-10000, Math.min(10000, value.height)) : defaults.height
    };
  }

  function makePlane(sceneToFixed, height) {
    // Column-major inverse of the display-follow transform. The shader's modelMatrix
    // includes both this follow transform and the cloud frame -> fixed frame TF.
    const m = sceneToFixed;
    return [m[2], m[6], m[10], m[14] - height];
  }

  function Provider({settings, follow, children}) {
    const {enabled, height} = normalize(settings);
    const value = React.useMemo(() => ({enabled,
      plane: makePlane(follow ? follow.clone().invert().elements : identity, height)
    }), [enabled, height, follow]);
    return h(Context.Provider, {value}, children);
  }

  function applyMaterial(material, clip) {
    // Apply to MITI's point and stixel shaders only, not meshes, labels or trajectories.
    if (!material.userData.rtabmapHeightClip) {
      const main = 'void main() {';
      if (!material.vertexShader.includes(main) || !material.fragmentShader.includes(main)) {
        throw new Error('Unsupported MITI point-cloud shader');
      }
      material.vertexShader = 'uniform vec4 rtabmapClipPlane;\nvarying float rtabmapClipDistance;\n' +
        material.vertexShader.replace(main, main +
          '\n  rtabmapClipDistance = dot(rtabmapClipPlane, modelMatrix * vec4(position, 1.0));');
      material.fragmentShader = 'uniform bool rtabmapClipEnabled;\nvarying float rtabmapClipDistance;\n' +
        material.fragmentShader.replace(main, main +
          '\n  if (rtabmapClipEnabled && rtabmapClipDistance > 0.0) discard;');
      material.uniforms.rtabmapClipPlane = {value: clip.plane};
      material.uniforms.rtabmapClipEnabled = {value: clip.enabled};
      material.userData.rtabmapHeightClip = true;
      material.needsUpdate = true;
    }
    material.uniforms.rtabmapClipPlane.value = clip.plane;
    material.uniforms.rtabmapClipEnabled.value = clip.enabled;
  }

  const css = `
    .rtab-height {position:absolute;top:8px;right:8px;z-index:3;max-width:calc(100% - 60px);
      color:#dce8f3;background:rgba(17,25,35,.94);border:1px solid #344355;border-radius:7px;
      box-shadow:0 3px 14px #0003;font:12px/1.45 system-ui,sans-serif;}
    .rtab-height * {box-sizing:border-box;}
    .rtab-height-toggle {display:flex;align-items:center;gap:8px;width:100%;padding:7px 10px;
      border:0;border-radius:6px;background:transparent;color:inherit;font:inherit;text-align:left;cursor:pointer;}
    .rtab-height-toggle[aria-pressed=true] {color:#77dfc0;}
    .rtab-height-indicator {width:7px;height:7px;border-radius:50%;background:#71839a;flex:none;}
    .rtab-height-toggle[aria-pressed=true] .rtab-height-indicator {background:#77dfc0;}
    .rtab-height-value {margin-left:auto;font-variant-numeric:tabular-nums;color:#9db0c3;}
    .rtab-height-fields {padding:0 10px 9px;width:270px;max-width:100%;}
    .rtab-height-label {display:block;font-size:11px;color:#a6b9cb;margin-bottom:5px;}
    .rtab-height-row {display:flex;align-items:center;gap:8px;}
    .rtab-height-range {flex:1;min-width:0;accent-color:#63d5b7;}
    .rtab-height-number {width:77px;min-width:0;padding:3px 4px;border:1px solid #445467;border-radius:4px;
      background:#101923;color:#e4edf5;font:inherit;font-variant-numeric:tabular-nums;}
    .rtab-height-note {margin-top:5px;color:#8ea3b8;font-size:10px;overflow-wrap:anywhere;}
    @media (pointer:coarse) {.rtab-height-toggle {min-height:44px}.rtab-height-number {font-size:16px;min-height:36px;}}
  `;

  function ControlsView({settings, frame, bounds, draft, id, onToggle, onSlide, onDraft, onCommit}) {
    return h('div', {className: 'rtab-height', role: 'group', 'aria-label': '点云顶部裁剪',
      onPointerDown: e => e.stopPropagation(), onPointerMove: e => e.stopPropagation(),
      onPointerUp: e => e.stopPropagation()},
      h('style', null, css),
      h('button', {type: 'button', className: 'rtab-height-toggle', onClick: onToggle,
        'aria-pressed': settings.enabled, 'aria-expanded': settings.enabled, 'aria-controls': id + '-fields'},
        h('span', {className: 'rtab-height-indicator', 'aria-hidden': true}), '隐藏顶部',
        h('span', {className: 'rtab-height-value'}, settings.enabled ? settings.height.toFixed(2) + ' m' : '关闭')),
      settings.enabled ? h('div', {className: 'rtab-height-fields', id: id + '-fields'},
        h('label', {className: 'rtab-height-label', htmlFor: id + '-range'}, '最高显示高度（米）'),
        h('div', {className: 'rtab-height-row'},
          h('input', {id: id + '-range', className: 'rtab-height-range', type: 'range',
            min: bounds[0], max: bounds[1], step: 0.05, value: settings.height, onChange: onSlide,
            'aria-label': '最高显示高度滑块'}),
          h('input', {className: 'rtab-height-number', type: 'number', step: 0.05,
            min: -10000, max: 10000, value: draft, onChange: onDraft, onBlur: onCommit,
            onKeyDown: e => {if (e.key === 'Enter') { e.preventDefault(); onCommit(); e.currentTarget.blur(); }},
            'aria-label': '最高显示高度数值（米）'})),
        h('div', {className: 'rtab-height-note'}, frame + ' 坐标系的 Z 高度，不代表离地高度。')) : null);
  }

  function Controls({settings: input, frame = 'map', onChange}) {
    const settings = normalize(input);
    const [draft, setDraft] = React.useState(() => settings.height.toFixed(2));
    const [bounds, setBounds] = React.useState(() => [Math.min(-5, Math.floor(settings.height - 1)),
                                                    Math.max(5, Math.ceil(settings.height + 1))]);
    const id = React.useId();
    React.useEffect(() => {
      setDraft(settings.height.toFixed(2));
      setBounds(previous => settings.height < previous[0] || settings.height > previous[1]
        ? [Math.min(previous[0], Math.floor(settings.height - 1)), Math.max(previous[1], Math.ceil(settings.height + 1))]
        : previous);
    }, [settings.height]);
    function commit() {
      const height = Number(draft);
      if (!draft.trim() || !Number.isFinite(height)) { setDraft(settings.height.toFixed(2)); return; }
      const next = normalize({...settings, height});
      setDraft(next.height.toFixed(2));
      onChange(next);
    }
    return h(ControlsView, {settings, frame, bounds, draft, id,
      onToggle: () => onChange({...settings, enabled: !settings.enabled}),
      onSlide: event => {const height = Number(event.target.value); onChange({...settings, height});},
      onDraft: event => setDraft(event.target.value), onCommit: commit});
  }

  return {Context, Provider, Controls, ControlsView, applyMaterial, normalize, makePlane};
}
