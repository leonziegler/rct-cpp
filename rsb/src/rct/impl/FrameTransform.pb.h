// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: FrameTransform.proto

#ifndef PROTOBUF_FrameTransform_2eproto__INCLUDED
#define PROTOBUF_FrameTransform_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "rst/geometry/Pose.pb.h"
#include "rst/timing/Timestamp.pb.h"
// @@protoc_insertion_point(includes)

namespace rct {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_FrameTransform_2eproto();
void protobuf_AssignDesc_FrameTransform_2eproto();
void protobuf_ShutdownFile_FrameTransform_2eproto();

class FrameTransform;

// ===================================================================

class FrameTransform : public ::google::protobuf::Message {
 public:
  FrameTransform();
  virtual ~FrameTransform();

  FrameTransform(const FrameTransform& from);

  inline FrameTransform& operator=(const FrameTransform& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const FrameTransform& default_instance();

  void Swap(FrameTransform* other);

  // implements Message ----------------------------------------------

  FrameTransform* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const FrameTransform& from);
  void MergeFrom(const FrameTransform& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .rst.geometry.Pose transform = 1;
  inline bool has_transform() const;
  inline void clear_transform();
  static const int kTransformFieldNumber = 1;
  inline const ::rst::geometry::Pose& transform() const;
  inline ::rst::geometry::Pose* mutable_transform();
  inline ::rst::geometry::Pose* release_transform();
  inline void set_allocated_transform(::rst::geometry::Pose* transform);

  // required string frame_parent = 2;
  inline bool has_frame_parent() const;
  inline void clear_frame_parent();
  static const int kFrameParentFieldNumber = 2;
  inline const ::std::string& frame_parent() const;
  inline void set_frame_parent(const ::std::string& value);
  inline void set_frame_parent(const char* value);
  inline void set_frame_parent(const char* value, size_t size);
  inline ::std::string* mutable_frame_parent();
  inline ::std::string* release_frame_parent();
  inline void set_allocated_frame_parent(::std::string* frame_parent);

  // required string frame_child = 3;
  inline bool has_frame_child() const;
  inline void clear_frame_child();
  static const int kFrameChildFieldNumber = 3;
  inline const ::std::string& frame_child() const;
  inline void set_frame_child(const ::std::string& value);
  inline void set_frame_child(const char* value);
  inline void set_frame_child(const char* value, size_t size);
  inline ::std::string* mutable_frame_child();
  inline ::std::string* release_frame_child();
  inline void set_allocated_frame_child(::std::string* frame_child);

  // optional .rst.timing.Timestamp time = 4;
  inline bool has_time() const;
  inline void clear_time();
  static const int kTimeFieldNumber = 4;
  inline const ::rst::timing::Timestamp& time() const;
  inline ::rst::timing::Timestamp* mutable_time();
  inline ::rst::timing::Timestamp* release_time();
  inline void set_allocated_time(::rst::timing::Timestamp* time);

  // @@protoc_insertion_point(class_scope:rct.FrameTransform)
 private:
  inline void set_has_transform();
  inline void clear_has_transform();
  inline void set_has_frame_parent();
  inline void clear_has_frame_parent();
  inline void set_has_frame_child();
  inline void clear_has_frame_child();
  inline void set_has_time();
  inline void clear_has_time();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::rst::geometry::Pose* transform_;
  ::std::string* frame_parent_;
  ::std::string* frame_child_;
  ::rst::timing::Timestamp* time_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(4 + 31) / 32];

  friend void  protobuf_AddDesc_FrameTransform_2eproto();
  friend void protobuf_AssignDesc_FrameTransform_2eproto();
  friend void protobuf_ShutdownFile_FrameTransform_2eproto();

  void InitAsDefaultInstance();
  static FrameTransform* default_instance_;
};
// ===================================================================


// ===================================================================

// FrameTransform

// optional .rst.geometry.Pose transform = 1;
inline bool FrameTransform::has_transform() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void FrameTransform::set_has_transform() {
  _has_bits_[0] |= 0x00000001u;
}
inline void FrameTransform::clear_has_transform() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void FrameTransform::clear_transform() {
  if (transform_ != NULL) transform_->::rst::geometry::Pose::Clear();
  clear_has_transform();
}
inline const ::rst::geometry::Pose& FrameTransform::transform() const {
  return transform_ != NULL ? *transform_ : *default_instance_->transform_;
}
inline ::rst::geometry::Pose* FrameTransform::mutable_transform() {
  set_has_transform();
  if (transform_ == NULL) transform_ = new ::rst::geometry::Pose;
  return transform_;
}
inline ::rst::geometry::Pose* FrameTransform::release_transform() {
  clear_has_transform();
  ::rst::geometry::Pose* temp = transform_;
  transform_ = NULL;
  return temp;
}
inline void FrameTransform::set_allocated_transform(::rst::geometry::Pose* transform) {
  delete transform_;
  transform_ = transform;
  if (transform) {
    set_has_transform();
  } else {
    clear_has_transform();
  }
}

// required string frame_parent = 2;
inline bool FrameTransform::has_frame_parent() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void FrameTransform::set_has_frame_parent() {
  _has_bits_[0] |= 0x00000002u;
}
inline void FrameTransform::clear_has_frame_parent() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void FrameTransform::clear_frame_parent() {
  if (frame_parent_ != &::google::protobuf::internal::kEmptyString) {
    frame_parent_->clear();
  }
  clear_has_frame_parent();
}
inline const ::std::string& FrameTransform::frame_parent() const {
  return *frame_parent_;
}
inline void FrameTransform::set_frame_parent(const ::std::string& value) {
  set_has_frame_parent();
  if (frame_parent_ == &::google::protobuf::internal::kEmptyString) {
    frame_parent_ = new ::std::string;
  }
  frame_parent_->assign(value);
}
inline void FrameTransform::set_frame_parent(const char* value) {
  set_has_frame_parent();
  if (frame_parent_ == &::google::protobuf::internal::kEmptyString) {
    frame_parent_ = new ::std::string;
  }
  frame_parent_->assign(value);
}
inline void FrameTransform::set_frame_parent(const char* value, size_t size) {
  set_has_frame_parent();
  if (frame_parent_ == &::google::protobuf::internal::kEmptyString) {
    frame_parent_ = new ::std::string;
  }
  frame_parent_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* FrameTransform::mutable_frame_parent() {
  set_has_frame_parent();
  if (frame_parent_ == &::google::protobuf::internal::kEmptyString) {
    frame_parent_ = new ::std::string;
  }
  return frame_parent_;
}
inline ::std::string* FrameTransform::release_frame_parent() {
  clear_has_frame_parent();
  if (frame_parent_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = frame_parent_;
    frame_parent_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void FrameTransform::set_allocated_frame_parent(::std::string* frame_parent) {
  if (frame_parent_ != &::google::protobuf::internal::kEmptyString) {
    delete frame_parent_;
  }
  if (frame_parent) {
    set_has_frame_parent();
    frame_parent_ = frame_parent;
  } else {
    clear_has_frame_parent();
    frame_parent_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// required string frame_child = 3;
inline bool FrameTransform::has_frame_child() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void FrameTransform::set_has_frame_child() {
  _has_bits_[0] |= 0x00000004u;
}
inline void FrameTransform::clear_has_frame_child() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void FrameTransform::clear_frame_child() {
  if (frame_child_ != &::google::protobuf::internal::kEmptyString) {
    frame_child_->clear();
  }
  clear_has_frame_child();
}
inline const ::std::string& FrameTransform::frame_child() const {
  return *frame_child_;
}
inline void FrameTransform::set_frame_child(const ::std::string& value) {
  set_has_frame_child();
  if (frame_child_ == &::google::protobuf::internal::kEmptyString) {
    frame_child_ = new ::std::string;
  }
  frame_child_->assign(value);
}
inline void FrameTransform::set_frame_child(const char* value) {
  set_has_frame_child();
  if (frame_child_ == &::google::protobuf::internal::kEmptyString) {
    frame_child_ = new ::std::string;
  }
  frame_child_->assign(value);
}
inline void FrameTransform::set_frame_child(const char* value, size_t size) {
  set_has_frame_child();
  if (frame_child_ == &::google::protobuf::internal::kEmptyString) {
    frame_child_ = new ::std::string;
  }
  frame_child_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* FrameTransform::mutable_frame_child() {
  set_has_frame_child();
  if (frame_child_ == &::google::protobuf::internal::kEmptyString) {
    frame_child_ = new ::std::string;
  }
  return frame_child_;
}
inline ::std::string* FrameTransform::release_frame_child() {
  clear_has_frame_child();
  if (frame_child_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = frame_child_;
    frame_child_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void FrameTransform::set_allocated_frame_child(::std::string* frame_child) {
  if (frame_child_ != &::google::protobuf::internal::kEmptyString) {
    delete frame_child_;
  }
  if (frame_child) {
    set_has_frame_child();
    frame_child_ = frame_child;
  } else {
    clear_has_frame_child();
    frame_child_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional .rst.timing.Timestamp time = 4;
inline bool FrameTransform::has_time() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void FrameTransform::set_has_time() {
  _has_bits_[0] |= 0x00000008u;
}
inline void FrameTransform::clear_has_time() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void FrameTransform::clear_time() {
  if (time_ != NULL) time_->::rst::timing::Timestamp::Clear();
  clear_has_time();
}
inline const ::rst::timing::Timestamp& FrameTransform::time() const {
  return time_ != NULL ? *time_ : *default_instance_->time_;
}
inline ::rst::timing::Timestamp* FrameTransform::mutable_time() {
  set_has_time();
  if (time_ == NULL) time_ = new ::rst::timing::Timestamp;
  return time_;
}
inline ::rst::timing::Timestamp* FrameTransform::release_time() {
  clear_has_time();
  ::rst::timing::Timestamp* temp = time_;
  time_ = NULL;
  return temp;
}
inline void FrameTransform::set_allocated_time(::rst::timing::Timestamp* time) {
  delete time_;
  time_ = time;
  if (time) {
    set_has_time();
  } else {
    clear_has_time();
  }
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace rct

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_FrameTransform_2eproto__INCLUDED