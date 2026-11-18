/* Stable per-object labels. Camera movement never groups or selects identities. */
function rtabmapSemanticLabelsFactory(React, Html) {
  'use strict';
  const h = React.createElement, Context = React.createContext('stable');
  const generic = new Set(['其他物品', '其他家具', '其他结构', 'otherprop', 'otherfurniture', 'otherstructure']);
  const contextual = new Set(['室内设计', '娱乐室', '院子', '武术馆', '汽车制造', '赛道',
    'home interior', 'recreation room', 'courtyard', 'martial arts gym', 'automobile make', 'race track']);
  const unnamed = new Set(['undefined', 'null', 'none', 'nan', 'unknown', 'unlabeled', 'unlabelled',
    '未知', '未知类别', '无标签', '未标注']);
  const isSemantic = marker => marker?.type === 9 && /^miti\/\d+\/label$/.test(marker.ns || '');
  const isAnchor = marker => marker?.type === 2 && /^miti\/\d+\/point$/.test(marker.ns || '');
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
    const text = typeof marker?.text === 'string'
      ? marker.text.replace(/[\u200B-\u200F\u2060\uFEFF]/g, '').trim() : '';
    // Suppressed or unnamed predictions must not leave an Html anchor, dot or
    // padded background behind. This also cleans retained older snapshots.
    if (!/[\p{L}\p{N}]/u.test(text) || unnamed.has(text.toLowerCase())) return null;
    if (mode === 'stable' && (generic.has(text) || contextual.has(text))) return null;
    const title = '预测类别：' + text + ' · 对象 #' + marker.ns.split('/')[1];
    return h(Html, {center: true, occlude: false, zIndexRange: [2, 0]},
      h('span', {title, 'aria-label': title, 'data-semantic-id': marker.ns,
        style: {color, opacity, whiteSpace: 'nowrap', pointerEvents: 'auto', display: 'inline-block',
          font: fontSize(labelScale) + 'px/1.4 system-ui,sans-serif',
          background: 'rgba(17,25,35,.82)', borderRadius: '4px',
          padding: '2px 5px', cursor: 'help'},
        onPointerDown: event => event.stopPropagation()}, text));
  }

  function Controls({settings, onChange}) {
    if (!settings.topics?.['/miti/semantic_markers'] || settings.topics['/miti/semantic_markers'].visible === false) return null;
    return h('label', {style: {display: 'flex', alignItems: 'center', gap: '8px', padding: '0 10px 7px'},
      title: '每个三维物体保留一个稳定类别；相邻独立物体分别显示。场景和泛化类别不显示占位标签，全部标签可查看原始类别。空白或无名称的记录不显示文字。'},
      '物体标签', h('select', {value: modeOf(settings.scene), 'aria-label': '物体标签显示方式',
        style: {background: '#101923', color: '#dce8f3', border: '1px solid #445467', borderRadius: '4px', font: 'inherit'},
        onChange: event => onChange({...settings, scene: {...settings.scene, rtabmapSemanticLabels: event.target.value}})},
        h('option', {value: 'stable'}, '稳定标签（单物体）'), h('option', {value: 'all'}, '全部标签'), h('option', {value: 'hidden'}, '隐藏标签')));
  }
  return {Provider, Label, Controls, isSemantic, isAnchor, modeOf, fontSize};
}
