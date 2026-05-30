import React, { useState, useEffect, useCallback, useRef } from 'react';
import { apiService } from '../services/api';
import './StoragePanel.css';

function StoragePanel({ isOpen, onClose, onNotification, refreshTrigger }) {
  const [activeTab, setActiveTab] = useState('photos');
  const [photos, setPhotos] = useState([]);
  const [loading, setLoading] = useState(false);
  const [previewItem, setPreviewItem] = useState(null);
  const [renameItem, setRenameItem] = useState(null);
  const [renameValue, setRenameValue] = useState('');
  const panelRef = useRef(null);

  const onNotifRef = useRef(onNotification);
  useEffect(() => {
    onNotifRef.current = onNotification;
  }, [onNotification]);

  const fetchFiles = useCallback(async () => {
    setLoading(true);
    try {
      const data = await apiService.getStorageFiles();

      const fetchedPhotos = data.filter(file => file.type === 'photo');

      setPhotos(fetchedPhotos.map(p => ({
        ...p,
        filename: p.name,
        size: p.size || 0,
        modified: p.time ? new Date(p.time * 1000).toLocaleString() : 'Unknown'
      })));

    } catch (e) {
      onNotifRef.current?.('Failed to load storage files', 'error');
      console.error(e);
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    if (isOpen) fetchFiles();
  }, [isOpen, fetchFiles, refreshTrigger]);

  // FIX: Kembalikan fungsi getFileURL untuk menerima 2 argumen: type dan filename
  const getFileURL = (type, filename) => {
    const folder = 'photos';
    return `${apiService.getBaseURL()}/api/storage/${folder}/${encodeURIComponent(filename)}`;
  };

  const handleDelete = async (filename) => {
    if (!window.confirm(`Delete "${filename}"?`)) return;
    try {
      const res = await fetch(
        `${apiService.getBaseURL()}/api/storage/delete/${encodeURIComponent(filename)}`,
        { method: 'DELETE' }
      );
      if (!res.ok) throw new Error(`Server returned ${res.status}`);
      const data = await res.json();
      if (data.success) {
        onNotifRef.current?.(`Deleted: ${filename}`, 'success');
        fetchFiles();
        if (previewItem?.filename === filename) setPreviewItem(null);
      } else {
        onNotifRef.current?.(data.error || 'Delete failed', 'error');
      }
    } catch (e) {
      onNotifRef.current?.('Delete request failed: ' + e.message, 'error');
    }
  };

  const openRename = (item) => {
    setRenameItem(item);
    setRenameValue(item.filename);
  };

  const handleRename = async () => {
    if (!renameItem) return;
    try {
      const res = await fetch(
        `${apiService.getBaseURL()}/api/storage/rename/${encodeURIComponent(renameItem.filename)}`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ newName: renameValue }),
        }
      );
      if (!res.ok) throw new Error(`Server returned ${res.status}`);
      const data = await res.json();
      if (data.success) {
        onNotifRef.current?.(`Renamed to: ${data.filename || renameValue}`, 'success');
        setRenameItem(null);
        fetchFiles();
      } else {
        onNotifRef.current?.(data.error || 'Rename failed', 'error');
      }
    } catch (e) {
      onNotifRef.current?.('Rename request failed: ' + e.message, 'error');
    }
  };

  const formatSize = (bytes) => {
    if (!bytes && bytes !== 0) return 'Unknown size';
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };

  const currentFiles = photos;

  if (!isOpen) return null;

  return (
    <>
      <div className="storage-backdrop" onClick={onClose} />
      <div className="storage-panel" ref={panelRef}>
        <div className="storage-header">
          <span className="storage-title">💾 Storage</span>
          <div className="storage-header-actions">
            <button className="storage-refresh-btn" onClick={fetchFiles} title="Refresh">🔄</button>
            <button className="storage-close-btn" onClick={onClose} title="Close">✕</button>
          </div>
        </div>

        <div className="storage-tabs">
          <button className="storage-tab active">
            📷 Photos ({photos.length})
          </button>
        </div>

        <div className="storage-list">
          {loading && <div className="storage-empty">Loading...</div>}
          {!loading && currentFiles.length === 0 && <div className="storage-empty">No {activeTab} stored yet.</div>}
          {!loading && currentFiles.map((file) => (
            <div key={file.filename} className="storage-item">
              <div className="storage-thumb" onClick={() => setPreviewItem(file)} title="Preview">
                <img src={getFileURL(file.type, file.filename)} alt={file.filename} loading="lazy" />
              </div>
              <div className="storage-info" onClick={() => setPreviewItem(file)} title="Preview">
                <div className="storage-filename">{file.filename}</div>
                <div className="storage-meta">{formatSize(file.size)} • {file.modified}</div>
              </div>
              <div className="storage-actions">
                <button className="storage-action-btn rename" onClick={(e) => { e.stopPropagation(); openRename(file); }} title="Rename">✏️</button>
                <button className="storage-action-btn delete" onClick={(e) => { e.stopPropagation(); handleDelete(file.filename); }} title="Delete">🗑️</button>
              </div>
            </div>
          ))}
        </div>
      </div>

      {previewItem && (
        <div className="storage-preview-overlay" onClick={() => setPreviewItem(null)}>
          <div className="storage-preview-modal" onClick={(e) => e.stopPropagation()}>
            <div className="storage-preview-header">
              <span>{previewItem.filename}</span>
              <div style={{ display: 'flex', gap: '0.5rem' }}>
                {/* FIX: Gunakan previewItem.type, previewItem.filename */}
                <a href={getFileURL(previewItem.type, previewItem.filename)} download={previewItem.filename} className="storage-action-btn rename" title="Download" style={{ textDecoration: 'none' }}>⬇️</a>
                <button className="storage-close-btn" onClick={() => setPreviewItem(null)}>✕</button>
              </div>
            </div>

            <div className="storage-preview-body" style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', background: '#111', minHeight: '300px' }}>
              <img
                src={getFileURL(previewItem.type, previewItem.filename)}
                alt={previewItem.filename}
                style={{ maxWidth: '100%', maxHeight: '70vh', borderRadius: '8px' }}
              />
            </div>

            <div className="storage-preview-footer">
              {formatSize(previewItem.size)} • {previewItem.modified}
            </div>
          </div>
        </div>
      )}

      {renameItem && (
        <div className="storage-preview-overlay" onClick={() => setRenameItem(null)}>
          <div className="storage-rename-dialog" onClick={(e) => e.stopPropagation()}>
            <div className="storage-preview-header">
              <span>Rename File</span>
              <button className="storage-close-btn" onClick={() => setRenameItem(null)}>✕</button>
            </div>
            <div style={{ padding: '1.25rem' }}>
              <div style={{ marginBottom: '0.5rem', fontSize: '0.85rem', opacity: 0.7 }}>Old: {renameItem.filename}</div>
              <input className="storage-rename-input" value={renameValue} onChange={(e) => setRenameValue(e.target.value)} onKeyDown={(e) => e.key === 'Enter' && handleRename()} autoFocus placeholder="New file name" />
              <div style={{ display: 'flex', gap: '0.75rem', marginTop: '1rem', justifyContent: 'flex-end' }}>
                <button className="btn btn-secondary" onClick={() => setRenameItem(null)}>Cancel</button>
                <button className="btn btn-primary" onClick={handleRename}>Rename</button>
              </div>
            </div>
          </div>
        </div>
      )}
    </>
  );
}

export default StoragePanel;