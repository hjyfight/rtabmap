/* Stable per-object labels. Camera movement never groups or selects identities. */
function rtabmapSemanticLabelsFactory(React, Html) {
  'use strict';
  const h = React.createElement, Context = React.createContext('stable');
  const generic = new Set(['其他物品', '其他家具', '其他结构', 'otherprop', 'otherfurniture', 'otherstructure']);
  const isSemantic = marker => marker?.type === 9 && /^miti\/\d+\/label$/.test(marker.ns || '');
  // Existing layouts may have saved "declutter". Retire that mode as well as
  // changing the default; a refresh must not keep the unstable old behavior.
  const modeOf = settings => ['all', 'hidden'].includes(settings?.rtabmapSemanticLabels)
    ? settings.rtabmapSemanticLabels : 'stable';
  // Html without distanceFactor keeps text in screen pixels. The original scene
  // labelScale=6 was tuned for perspective scaling; bound it for semantic text.
  const fontSize = scale => Math.max(10, Math.min(16, 12 * Math.sqrt(
    Number.isFinite(scale) && scale > 0 ? scale : 1)));

  function Provider({settings, children}) {
    return h(Context.Provider, {value: modeOf(settings)}, children);
  }

  function Label({marker, color, opacity, labelScale}) {
    const mode = React.useContext(Context);
    if (mode === 'hidden') return null;
    const compact = mode === 'stable' && generic.has(marker.text);
    const title = '预测类别：' + marker.text + ' · 对象 #' + marker.ns.split('/')[1];
    return h(Html, {center: true, occlude: false, zIndexRange: [2, 0]},
      h('span', {title, 'aria-label': title, 'data-semantic-id': marker.ns,
        style: {color, opacity, whiteSpace: 'nowrap', pointerEvents: 'auto', display: 'inline-block',
          font: fontSize(labelScale) + 'px/1.4 system-ui,sans-serif',
          background: 'rgba(17,25,35,.82)', borderRadius: compact ? '50%' : '4px',
          padding: compact ? '0 4px' : '2px 5px', cursor: 'help'},
        onPointerDown: event => event.stopPropagation()}, compact ? '·' : marker.text));
  }

  function Controls({settings, onChange}) {
    if (!settings.topics?.['/miti/semantic_markers'] || settings.topics['/miti/semantic_markers'].visible === false) return null;
    return h('label', {style: {display: 'flex', alignItems: 'center', gap: '8px', padding: '0 10px 7px'},
      title: '稳定标签显示具体类别；其他物品、其他家具等用小圆点表示，悬停查看。旋转视角不改变标签内容。'},
      '物体标签', h('select', {value: modeOf(settings.scene), 'aria-label': '物体标签显示方式',
        style: {background: '#101923', color: '#dce8f3', border: '1px solid #445467', borderRadius: '4px', font: 'inherit'},
        onChange: event => onChange({...settings, scene: {...settings.scene, rtabmapSemanticLabels: event.target.value}})},
        h('option', {value: 'stable'}, '稳定标签'), h('option', {value: 'all'}, '全部标签'), h('option', {value: 'hidden'}, '隐藏标签')));
  }
  return {Provider, Label, Controls, isSemantic, modeOf, fontSize};
}
